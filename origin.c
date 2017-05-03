/***********************************************************************/
/***********************************************************************
 Pipeline Cache Simulator
 ***********************************************************************/
/***********************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define MAX_CACHE_SIZE 10240
#define CACHE_MISS_DELAY 10 // 10 cycle cache miss penalty
#define MAX_STAGES 5

// init the simulator
void iplc_sim_init(int index, int blocksize, int assoc);

// Cache simulator functions
void iplc_sim_LRU_replace_on_miss(int index, int tag);
void iplc_sim_LRU_update_on_hit(int index, int assoc);
int iplc_sim_trap_address(unsigned int address);

// Pipeline functions
unsigned int iplc_sim_parse_reg(char *reg_str);
void iplc_sim_parse_instruction(char *buffer);
void iplc_sim_push_pipeline_stage();
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg,
                                     int reg1, int reg2_or_constant);
void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_branch(int reg1, int reg2);
void iplc_sim_process_pipeline_jump();
void iplc_sim_process_pipeline_syscall();
void iplc_sim_process_pipeline_nop();

// Outout performance results
void iplc_sim_finalize();

typedef struct cache_line
{
    /* the struct contains 3 arrays - one to contain valid bits, 
    one to contain tag data, and one to contain the time each entry was accessed*/
    int* valid; 
    int* tag;
    int* time;
} cache_line_t;

cache_line_t *cache=NULL;
int cache_index=0;
int cache_blocksize=0;
int cache_blockoffsetbits = 0;
int cache_assoc=0;
long cache_miss=0;
long cache_access=0;
long cache_hit=0;

char instruction[16];
char reg1[16];
char reg2[16];
char offsetwithreg[16];
unsigned int data_address=0;
unsigned int instruction_address=0;
unsigned int pipeline_cycles=0;   // how many cycles did you pipeline consume
unsigned int instruction_count=0; // home many real instructions ran thru the pipeline
unsigned int branch_predict_taken=0;
unsigned int branch_count=0;
unsigned int correct_branch_predictions=0;

unsigned int debug=0;
unsigned int dump_pipeline=1;

enum instruction_type {NOP, RTYPE, LW, SW, BRANCH, JUMP, JAL, SYSCALL};

typedef struct rtype
{
    char instruction[16];
    int reg1;
    int reg2_or_constant;
    int dest_reg;
    
} rtype_t;

typedef struct load_word
{
    unsigned int data_address;
    int dest_reg;
    int base_reg;
    
} lw_t;

typedef struct store_word
{
    unsigned int data_address;
    int src_reg;
    int base_reg;
} sw_t;

typedef struct branch
{
    int reg1;
    int reg2;
    
} branch_t;


typedef struct jump
{
    char instruction[16];
    
} jump_t;

typedef struct pipeline
{
    enum instruction_type itype;
    unsigned int instruction_address;
    union
    {
        rtype_t   rtype;
        lw_t      lw;
        sw_t      sw;
        branch_t  branch;
        jump_t    jump;
    }
    stage;
    
} pipeline_t;

enum pipeline_stages {FETCH, DECODE, ALU, MEM, WRITEBACK};

pipeline_t pipeline[MAX_STAGES];

/************************************************************************************************/
/* Cache Functions ******************************************************************************/
/************************************************************************************************/
/*
 * Correctly configure the cache.
 */
void iplc_sim_init(int index, int blocksize, int assoc)
{
    int i=0, j=0;
    unsigned long cache_size = 0;
    cache_index = index;
    cache_blocksize = blocksize;
    cache_assoc = assoc;
    
    
    cache_blockoffsetbits =
    (int) rint((log( (double) (blocksize * 4) )/ log(2)));
    /* Note: rint function rounds the result up prior to casting */
    
    cache_size = assoc * ( 1 << index ) * ((32 * blocksize) + 33 - index - cache_blockoffsetbits);
    
    printf("Cache Configuration \n");
    printf("   Index: %d bits or %d lines \n", cache_index, (1<<cache_index) );
    printf("   BlockSize: %d \n", cache_blocksize );
    printf("   Associativity: %d \n", cache_assoc );
    printf("   BlockOffSetBits: %d \n", cache_blockoffsetbits );
    printf("   CacheSize: %lu \n", cache_size );
    
    if (cache_size > MAX_CACHE_SIZE ) {
        printf("Cache too big. Great than MAX SIZE of %d .... \n", MAX_CACHE_SIZE);
        exit(-1);
    }
    
    cache = (cache_line_t *) calloc((1 << index), sizeof(cache_line_t));
    
    // Dynamically create our cache based on the information the user entered
    /*the array is initialized with calloc to set everything to 0 */
    for (i = 0; i < (1<<index); i++) {
        cache[i].valid = (int *) calloc(cache_assoc, sizeof(int));
        cache[i].tag = (int *) calloc(cache_assoc, sizeof(int));
        cache[i].time = (int *) calloc(cache_assoc, sizeof(int));
        for(j = 0; j < cache_assoc; j++){
            cache[i].valid[j] = 0;
            cache[i].tag[j] = 0;
            cache[i].time[j] = 0;
        }
    }

    
    // init the pipeline -- set all data to zero and instructions to NOP
    for (i = 0; i < MAX_STAGES; i++) {
        // itype is set to O which is NOP type instruction
        bzero(&(pipeline[i]), sizeof(pipeline_t));
    }
}

/*
 * iplc_sim_trap_address() determined this is not in our cache.  Put it there
 * and make sure that is now our Most Recently Used (MRU) entry.
 */
void iplc_sim_LRU_replace_on_miss(int index, int tag)
{
    cache_miss++;
    /*the loop checks if any valid bits are open, if so,
    place tag in the spot and end function */
    for(int i = 0; i < cache_assoc; i++){
        if(cache[index].valid[i] == 0){
            cache[index].valid[i] = 1;
            cache[index].time[i] = cache_access;
            cache[index].tag[i] = tag;
            return;
        }
    }
    /*counter checks for the element that was used longest ago,
    then that element is replaced */
    int counter = 0;
    for(int i = 0; i < cache_assoc; i++){
        if(cache[index].time[i] < cache[index].time[counter])
            counter = i;
    }
    cache[index].valid[counter] = 1;
    cache[index].time[counter] = cache_access;
    cache[index].tag[counter] = tag;
}

/*
 * iplc_sim_trap_address() determined the entry is in our cache.  Update its
 * information in the cache.
 */
void iplc_sim_LRU_update_on_hit(int index, int assoc_entry)
{
    cache[index].time[assoc_entry] = cache_access;
    cache_hit++;
}

/*
 * Check if the address is in our cache.  Update our counter statistics 
 * for cache_access, cache_hit, etc.  If our configuration supports
 * associativity we may need to check through multiple entries for our
 * desired index.  In that case we will also need to call the LRU functions.
 */
int iplc_sim_trap_address(unsigned int address)
{
    int i=0, index=0;
    int tag=0;
    int hit=0;

    cache_access++;
    index = (address >> cache_blockoffsetbits) & ((1 << cache_index) - 1);
    tag = (address >> cache_blockoffsetbits) >> cache_index;
    // Call the appropriate function for a miss or hit
    for(i = 0; i < cache_assoc; i++){
        if(cache[index].valid[i] == 1 && cache[index].tag[i] == tag){
            iplc_sim_LRU_update_on_hit(index, i);
            hit = 1;
            break;
        }
    }
    if(hit == 0){
        iplc_sim_LRU_replace_on_miss(index, tag);
    }
    /* expects you to return 1 for hit, 0 for miss */
    return hit;
}

/*
 * Just output our summary statistics.
 */
void iplc_sim_finalize()
{
    /* Finish processing all instructions in the Pipeline */
    while (pipeline[FETCH].itype != NOP  ||
           pipeline[DECODE].itype != NOP ||
           pipeline[ALU].itype != NOP    ||
           pipeline[MEM].itype != NOP    ||
           pipeline[WRITEBACK].itype != NOP) {
        iplc_sim_push_pipeline_stage();
    }
    
    printf(" Cache Performance \n");
    printf("\t Number of Cache Accesses is %ld \n", cache_access);
    printf("\t Number of Cache Misses is %ld \n", cache_miss);
    printf("\t Number of Cache Hits is %ld \n", cache_hit);
    printf("\t Cache Miss Rate is %f \n\n", (double)cache_miss / (double)cache_access);
    printf("Pipeline Performance \n");
    printf("\t Total Cycles is %u \n", pipeline_cycles);
    printf("\t Total Instructions is %u \n", instruction_count);
    printf("\t Total Branch Instructions is %u \n", branch_count);
    printf("\t Total Correct Branch Predictions is %u \n", correct_branch_predictions);
    printf("\t CPI is %f \n\n", (double)pipeline_cycles / (double)instruction_count);
}

/************************************************************************************************/
/* Pipeline Functions ***************************************************************************/
/************************************************************************************************/

/*
 * Dump the current contents of our pipeline.
 */
void iplc_sim_dump_pipeline()
{
    int i;
    for (i = 0; i < MAX_STAGES; i++) {
        switch(i) {
            case FETCH:
                printf("(cyc: %u) FETCH:\t %d: 0x%x \t", pipeline_cycles, pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case DECODE:
                printf("DECODE:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case ALU:
                printf("ALU:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case MEM:
                printf("MEM:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case WRITEBACK:
                printf("WB:\t %d: 0x%x \n", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            default:
                printf("DUMP: Bad stage!\n" );
                exit(-1);
        }
    }
}

/*
 * Check if various stages of our pipeline require stalls, forwarding, etc.
 * Then push the contents of our various pipeline stages through the pipeline.
 */
void iplc_sim_push_pipeline_stage()
{
    //int i;
    //int data_hit=1;
    
    /* 1. Count WRITEBACK stage is "retired" -- This I'm giving you */
    if (pipeline[WRITEBACK].instruction_address) {
        instruction_count++;
        if (debug)
            printf("DEBUG: Retired Instruction at 0x%x, Type %d, at Time %u \n",
                   pipeline[WRITEBACK].instruction_address, pipeline[WRITEBACK].itype, pipeline_cycles);
    }
    
    /* 2. Check for BRANCH and correct/incorrect Branch Prediction */
    if (pipeline[DECODE].itype == BRANCH) {
        int branch_taken = (pipeline[FETCH].instruction_address > (pipeline[DECODE].instruction_address + 4));

        branch_count++;

        if(pipeline[FETCH].instruction_address){
            if(branch_taken == branch_predict_taken){
                correct_branch_predictions++;
            } else {
                pipeline_cycles += 1;
            }
        }
        //instruction address is not a null.
    }
    
    /* 3. Check for LW delays due to use in ALU stage and if data hit/miss
     *    add delay cycles if needed.
     */
    if (pipeline[MEM].itype == LW) {
        //int inserted_nop = 0;
        
        int hit = iplc_sim_trap_address(pipeline[MEM].stage.lw.data_address);

        if(!hit){ //If the ALU is full
            pipeline_cycles += 9;
        }


    }
    
    /* 4. Check for SW mem acess and data miss .. add delay cycles if needed */
    if (pipeline[MEM].itype == SW) {
        int hit = iplc_sim_trap_address(pipeline[MEM].stage.sw.data_address);

        if(!hit){
            pipeline_cycles += 9;
        }
    }
    
    /* 5. Increment pipe_cycles 1 cycle for normal processing */
    pipeline_cycles++;

    /* 6. push stages thru MEM->WB, ALU->MEM, DECODE->ALU, FETCH->ALU */
    for(int i = WRITEBACK; i > FETCH; i--){
        pipeline[i].itype = pipeline[i-1].itype;
        pipeline[i].instruction_address = pipeline[i-1].instruction_address;
        pipeline[i].stage = pipeline[i-1].stage;
    }
    //push_forward method to dump contents from pipeline[0] into pipeline[1], etc...

    // 7. This is a give'me -- Reset the FETCH stage to NOP via bezero */
    bzero(&(pipeline[FETCH]), sizeof(pipeline_t));
    
}

/*
 * This function is fully implemented.  You should use this as a reference
 * for implementing the remaining instruction types.
 */
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg, int reg1, int reg2_or_constant)
{
    /* This is an example of what you need to do for the rest */
    iplc_sim_push_pipeline_stage();
    
    pipeline[FETCH].itype = RTYPE;
    pipeline[FETCH].instruction_address = instruction_address;
    
    strcpy(pipeline[FETCH].stage.rtype.instruction, instruction);
    pipeline[FETCH].stage.rtype.reg1 = reg1;
    pipeline[FETCH].stage.rtype.reg2_or_constant = reg2_or_constant;
    pipeline[FETCH].stage.rtype.dest_reg = dest_reg;
}

void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = LW;
    pipeline[FETCH].instruction_address = instruction_address;

    /*
    unsigned int data_address;
    int dest_reg;    int base_reg;

    */

    pipeline[FETCH].stage.lw.data_address = data_address;
    pipeline[FETCH].stage.lw.dest_reg = dest_reg;
    pipeline[FETCH].stage.lw.base_reg = base_reg;


}

void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SW;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.sw.data_address = data_address;
    pipeline[FETCH].stage.sw.src_reg = src_reg;
    pipeline[FETCH].stage.sw.base_reg = base_reg;
}

void iplc_sim_process_pipeline_branch(int reg1, int reg2)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = BRANCH;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.branch.reg1 = reg1;
    pipeline[FETCH].stage.branch.reg2 = reg2;
}

void iplc_sim_process_pipeline_jump(char *instruction)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = JUMP;
    pipeline[FETCH].instruction_address = instruction_address;

    strcpy(pipeline[FETCH].stage.jump.instruction, instruction);
}

void iplc_sim_process_pipeline_syscall()
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SYSCALL;
    pipeline[FETCH].instruction_address = instruction_address;
}

void iplc_sim_process_pipeline_nop()
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();
    
    pipeline[FETCH].itype = NOP;
    pipeline[FETCH].instruction_address = instruction_address;
}

/************************************************************************************************/
/* parse Function *******************************************************************************/
/************************************************************************************************/

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
unsigned int iplc_sim_parse_reg(char *reg_str)
{
    int i;
    // turn comma into \n
    if (reg_str[strlen(reg_str)-1] == ',')
        reg_str[strlen(reg_str)-1] = '\n';
    
    if (reg_str[0] != '$')
        return atoi(reg_str);
    else {
        // copy down over $ character than return atoi
        for (i = 0; i < strlen(reg_str); i++)
            reg_str[i] = reg_str[i+1];
        
        return atoi(reg_str);
    }
}

/*
 * Don't touch this function.  It is for parsing the instruction stream.
 */
void iplc_sim_parse_instruction(char *buffer)
{
    int instruction_hit = 0;
    int i=0, j=0;
    int src_reg=0;
    int src_reg2=0;
    int dest_reg=0;
    char str_src_reg[16];
    char str_src_reg2[16];
    char str_dest_reg[16];
    char str_constant[16];
    
    if (sscanf(buffer, "%x %s", &instruction_address, instruction ) != 2) {
        printf("Malformed instruction \n");
        exit(-1);
    }
    
    instruction_hit = iplc_sim_trap_address( instruction_address );
    
    // if a MISS, then push current instruction thru pipeline
    if (!instruction_hit) {
        // need to subtract 1, since the stage is pushed once more for actual instruction processing
        // also need to allow for a branch miss prediction during the fetch cache miss time -- by
        // counting cycles this allows for these cycles to overlap and not doubly count.
    
        printf("INST MISS:\t Address 0x%x \n", instruction_address);
        
        for (i = pipeline_cycles, j = pipeline_cycles; i < j + CACHE_MISS_DELAY - 1; i++)
            iplc_sim_push_pipeline_stage();
    }
    else
            printf("INST HIT:\t Address 0x%x \n", instruction_address);
    
    // Parse the Instruction
    
    if (strncmp( instruction, "add", 3 ) == 0 ||
        strncmp( instruction, "sll", 3 ) == 0 ||
        strncmp( instruction, "ori", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_src_reg,
                   str_src_reg2 ) != 5) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",instruction, instruction_address);
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = iplc_sim_parse_reg(str_src_reg);
        src_reg2 = iplc_sim_parse_reg(str_src_reg2);
        
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lui", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_constant ) != 4 ) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",instruction, instruction_address );
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = -1;
        src_reg2 = -1;
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lw", 2 ) == 0 ||
             strncmp( instruction, "sw", 2 ) == 0  ) {
        if ( sscanf( buffer, "%x %s %s %s %x",
                    &instruction_address,
                    instruction,
                    reg1,
                    offsetwithreg,
                    &data_address ) != 5) {
            printf("Bad instruction: %s at address %x \n", instruction, instruction_address);
            exit(-1);
        }
        
        if (strncmp(instruction, "lw", 2 ) == 0) {
            
            dest_reg = iplc_sim_parse_reg(reg1);
            
            // don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_lw(dest_reg, -1, data_address);
        }
        if (strncmp( instruction, "sw", 2 ) == 0) {
            src_reg = iplc_sim_parse_reg(reg1);
            
            // don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_sw( src_reg, -1, data_address);
        }
    }
    else if (strncmp( instruction, "beq", 3 ) == 0) {
        // don't need to worry about getting regs -- just insert -1 values
        iplc_sim_process_pipeline_branch(-1, -1);
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        iplc_sim_process_pipeline_jump( instruction );
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        /*
         * Note: no need to worry about forwarding on the jump register
         * we'll let that one go.
         */
        iplc_sim_process_pipeline_jump(instruction);
    }
    else if ( strncmp( instruction, "syscall", 7 ) == 0) {
        iplc_sim_process_pipeline_syscall( );
    }
    else if ( strncmp( instruction, "nop", 3 ) == 0) {
        iplc_sim_process_pipeline_nop( );
    }
    else {
        printf("Do not know how to process instruction: %s at address %x \n",
               instruction, instruction_address );
        exit(-1);
    }
}

/************************************************************************************************/
/* MAIN Function ********************************************************************************/
/************************************************************************************************/

int main()
{
    char trace_file_name[1024];
    FILE *trace_file = NULL;
    char buffer[80];
    int index = 10;
    int blocksize = 1;
    int assoc = 1;
    
    printf("Please enter the tracefile: ");
    scanf("%s", trace_file_name);
    
    trace_file = fopen(trace_file_name, "r");
    
    if ( trace_file == NULL ) {
        printf("fopen failed for %s file\n", trace_file_name);
        exit(-1);
    }
    
    printf("Enter Cache Size (index), Blocksize and Level of Assoc \n");
    scanf( "%d %d %d", &index, &blocksize, &assoc );
    
    printf("Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): ");
    scanf("%d", &branch_predict_taken );
    
    iplc_sim_init(index, blocksize, assoc);
    
    while (fgets(buffer, 80, trace_file) != NULL) {
        iplc_sim_parse_instruction(buffer);
        if (dump_pipeline)
            iplc_sim_dump_pipeline();
    }
    
    iplc_sim_finalize();
    return 0;
}
/*
CacheSize(index): 7

Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 7 bits or 128 lines 
   BlockSize: 1 
   Associativity: 1 
   BlockOffSetBits: 2 
   CacheSize: 7168 
 Cache Performance 
     Number of Cache Accesses is 35676 
     Number of Cache Misses is 1113 
     Number of Cache Hits is 34563 
     Cache Miss Rate is 0.031197 
Pipeline Performance 
     Total Cycles is 45419 
     Total Instructions is 34129 
     Total Branch Instructions is 7020 
     Total Correct Branch Predictions is 5730 
     CPI is 1.330804 


Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 1 
   Associativity: 2 
   BlockOffSetBits: 2 
   CacheSize: 7296 
 Cache Performance 
     Number of Cache Accesses is 35780 
     Number of Cache Misses is 480 
     Number of Cache Hits is 35300 
     Cache Miss Rate is 0.013415 
Pipeline Performance 
     Total Cycles is 40049 
     Total Instructions is 34449 
     Total Branch Instructions is 7031 
     Total Correct Branch Predictions is 5734 
     CPI is 1.162559 


Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 1 
   Associativity: 4 
   BlockOffSetBits: 2 
   CacheSize: 7424 
 Cache Performance 
     Number of Cache Accesses is 35823 
     Number of Cache Misses is 349 
     Number of Cache Hits is 35474 
     Cache Miss Rate is 0.009742 
Pipeline Performance 
     Total Cycles is 38975 
     Total Instructions is 34550 
     Total Branch Instructions is 7036 
     Total Correct Branch Predictions is 5744 
     CPI is 1.128075 


2 Word, Direct Mapped
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 2 
   Associativity: 1 
   BlockOffSetBits: 3 
   CacheSize: 5632 
 Cache Performance 
     Number of Cache Accesses is 35691 
     Number of Cache Misses is 1027 
     Number of Cache Hits is 34664 
     Cache Miss Rate is 0.028775 
Pipeline Performance 
     Total Cycles is 44738 
     Total Instructions is 34212 
     Total Branch Instructions is 7015 
     Total Correct Branch Predictions is 5714 
     CPI is 1.307670 


2 Word, 2-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 2 
   Associativity: 2 
   BlockOffSetBits: 3 
   CacheSize: 5696 
 Cache Performance 
     Number of Cache Accesses is 35793 
     Number of Cache Misses is 349 
     Number of Cache Hits is 35444 
     Cache Miss Rate is 0.009751 

Pipeline Performance 
     Total Cycles is 38982 
     Total Instructions is 34551 
     Total Branch Instructions is 7042 
     Total Correct Branch Predictions is 5734 
     CPI is 1.128245


2 Word, 4-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 4 bits or 16 lines 
   BlockSize: 2 
   Associativity: 4 
   BlockOffSetBits: 3 
   CacheSize: 5760 
 Cache Performance 
     Number of Cache Accesses is 35841 
     Number of Cache Misses is 197 
     Number of Cache Hits is 35644 
     Cache Miss Rate is 0.005496 

Pipeline Performance 
     Total Cycles is 37710 
     Total Instructions is 34647 
     Total Branch Instructions is 7042 
     Total Correct Branch Predictions is 5744 
     CPI is 1.088406 


4 Word, Direct Mapped
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 4 
   Associativity: 1 
   BlockOffSetBits: 4 
   CacheSize: 9664 
 Cache Performance 
     Number of Cache Accesses is 35835 
     Number of Cache Misses is 750 
     Number of Cache Hits is 35085 
     Cache Miss Rate is 0.020929 

Pipeline Performance 
     Total Cycles is 42396 
     Total Instructions is 34362 
     Total Branch Instructions is 6997 
     Total Correct Branch Predictions is 5689 
     CPI is 1.233805 


4 Word, 2-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 4 
   Associativity: 2 
   BlockOffSetBits: 4 
   CacheSize: 9728 
 Cache Performance 
     Number of Cache Accesses is 35855 
     Number of Cache Misses is 144 
     Number of Cache Hits is 35711 
     Cache Miss Rate is 0.004016 
Pipeline Performance 
     Total Cycles is 37268 
     Total Instructions is 34681 
     Total Branch Instructions is 7043 
     Total Correct Branch Predictions is 5748 
     CPI is 1.074594 


4 Word, 4-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 0
Cache Configuration 
   Index: 4 bits or 16 lines 
   BlockSize: 4 
   Associativity: 4 
   BlockOffSetBits: 4 
   CacheSize: 9792 
 Cache Performance 
     Number of Cache Accesses is 35855 
     Number of Cache Misses is 78 
     Number of Cache Hits is 35777 
     Cache Miss Rate is 0.002175 
Pipeline Performance 
     Total Cycles is 36694 
     Total Instructions is 34701 
     Total Branch Instructions is 7043 
     Total Correct Branch Predictions is 5748 
     CPI is 1.057434 


NOT Taken: 1
1 Word, Direct Mapped
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 7 bits or 128 lines 
   BlockSize: 1 
   Associativity: 1 
   BlockOffSetBits: 2 
   CacheSize: 7168 
 Cache Performance 
     Number of Cache Accesses is 35676 
     Number of Cache Misses is 1113 
     Number of Cache Hits is 34563 
     Cache Miss Rate is 0.031197 
Pipeline Performance 
     Total Cycles is 49876 
     Total Instructions is 34129 
     Total Branch Instructions is 7020 
     Total Correct Branch Predictions is 1273 
     CPI is 1.461396 

1 Word, 2-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 1 
   Associativity: 2 
   BlockOffSetBits: 2 
   CacheSize: 7296 
 Cache Performance 
     Number of Cache Accesses is 35780 
     Number of Cache Misses is 480 
     Number of Cache Hits is 35300 
     Cache Miss Rate is 0.013415 
Pipeline Performance 
     Total Cycles is 44503 
     Total Instructions is 34449 
     Total Branch Instructions is 7031 
     Total Correct Branch Predictions is 1280 
     CPI is 1.291852

1 Word, 4-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 1 
   Associativity: 4 
   BlockOffSetBits: 2 
   CacheSize: 7424 
 Cache Performance 
     Number of Cache Accesses is 35823 
     Number of Cache Misses is 349 
     Number of Cache Hits is 35474 
     Cache Miss Rate is 0.009742 
Pipeline Performance 
     Total Cycles is 43435 
     Total Instructions is 34550 
     Total Branch Instructions is 7036 
     Total Correct Branch Predictions is 1284 
     CPI is 1.257164 


2 Word, Direct Mapped
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 2 
   Associativity: 1 
   BlockOffSetBits: 3 
   CacheSize: 5632 
 Cache Performance 
     Number of Cache Accesses is 35691 
     Number of Cache Misses is 1027 
     Number of Cache Hits is 34664 
     Cache Miss Rate is 0.028775 
Pipeline Performance 
     Total Cycles is 49169 
     Total Instructions is 34212 
     Total Branch Instructions is 7015 
     Total Correct Branch Predictions is 1283 
     CPI is 1.437186 

2 Word, 2-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 2 
   Associativity: 2 
   BlockOffSetBits: 3 
   CacheSize: 5696 
 Cache Performance 
     Number of Cache Accesses is 35793 
     Number of Cache Misses is 349 
     Number of Cache Hits is 35444 
     Cache Miss Rate is 0.009751 
Pipeline Performance 
     Total Cycles is 43426 
     Total Instructions is 34551 
     Total Branch Instructions is 7042 
     Total Correct Branch Predictions is 1290 
     CPI is 1.256867 

2 Word, 4-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 4 bits or 16 lines 
   BlockSize: 2 
   Associativity: 4 
   BlockOffSetBits: 3 
   CacheSize: 5760 
 Cache Performance 
     Number of Cache Accesses is 35841 
     Number of Cache Misses is 197 
     Number of Cache Hits is 35644 
     Cache Miss Rate is 0.005496 
Pipeline Performance 
     Total Cycles is 42164 
     Total Instructions is 34647 
     Total Branch Instructions is 7042 
     Total Correct Branch Predictions is 1290 
     CPI is 1.216960 


4 Word, Direct Mapped
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 6 bits or 64 lines 
   BlockSize: 4 
   Associativity: 1 
   BlockOffSetBits: 4 
   CacheSize: 9664 
 Cache Performance 
     Number of Cache Accesses is 35835 
     Number of Cache Misses is 750 
     Number of Cache Hits is 35085 
     Cache Miss Rate is 0.020929 
Pipeline Performance 
     Total Cycles is 46801 
     Total Instructions is 34362 
     Total Branch Instructions is 6997 
     Total Correct Branch Predictions is 1284 
     CPI is 1.361999 



4 Word, 2-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 5 bits or 32 lines 
   BlockSize: 4 
   Associativity: 2 
   BlockOffSetBits: 4 
   CacheSize: 9728 
 Cache Performance 
     Number of Cache Accesses is 35855 
     Number of Cache Misses is 144 
     Number of Cache Hits is 35711 
     Cache Miss Rate is 0.004016 
Pipeline Performance 
     Total Cycles is 41725 
     Total Instructions is 34681 
     Total Branch Instructions is 7043 
     Total Correct Branch Predictions is 1291 
     CPI is 1.203108


4 Word, 4-Way
Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): 1
Cache Configuration 
   Index: 4 bits or 16 lines 
   BlockSize: 4 
   Associativity: 4 
   BlockOffSetBits: 4 
   CacheSize: 9792 
 Cache Performance 
     Number of Cache Accesses is 35855 
     Number of Cache Misses is 78 
     Number of Cache Hits is 35777 
     Cache Miss Rate is 0.002175 
Pipeline Performance 
     Total Cycles is 41151 
     Total Instructions is 34701 
     Total Branch Instructions is 7043 
     Total Correct Branch Predictions is 1291 
     CPI is 1.185874 
*/
