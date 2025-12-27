    #include <unistd.h>
    #include <stdio.h>
    #include <stdint.h>

    // if using CPUlator, you should copy+paste contents of the file below instead of using #include
    #include "address_map_niosv.h"


    typedef uint16_t pixel_t;

    volatile pixel_t *pVGA = (pixel_t *)FPGA_PIXEL_BUF_BASE;


    const pixel_t blk = 0x0000;
    const pixel_t wht = 0xffff;
    const pixel_t red = 0xf800;
    const pixel_t grn = 0x07e0;
    const pixel_t blu = 0x001f;

    /* FIXED: proper MTIME + MTIMECMP macros for Nios V ACLINT */
    #define MTIMECMP_BASE   (MTIME_BASE + 8)

    #define MTIME_LOW        (*(volatile uint32_t *)(MTIME_BASE))          // lower 32b of mtime
    #define MTIME_HIGH       (*(volatile uint32_t *)(MTIME_BASE + 4))      // upper 32b of mtime
    #define MTIMECMP_LOW     (*(volatile uint32_t *)(MTIMECMP_BASE))       // lower 32b of mtimecmp
    #define MTIMECMP_HIGH    (*(volatile uint32_t *)(MTIMECMP_BASE + 4))   // upper 32b of mtimecmp

    /* BASE PERIOD */
    #define BASE_TICKS 50000U // about 1 ms at 50MHz

    /* DIRECTIONS */
    #define DIR_UP      0
    #define DIR_RIGHT   1
    #define DIR_DOWN    2
    #define DIR_LEFT    3


    void drawPixel( int y, int x, pixel_t colour )
    {
        *(pVGA + (y<<YSHIFT) + x ) = colour;
    }

    pixel_t makePixel( uint8_t r8, uint8_t g8, uint8_t b8 )
    {
        // inputs: 8b of each: red, green, blue
        const uint16_t r5 = (r8 & 0xf8)>>3; // keep 5b red
        const uint16_t g6 = (g8 & 0xfc)>>2; // keep 6b green
        const uint16_t b5 = (b8 & 0xf8)>>3; // keep 5b blue
        return (pixel_t)( (r5<<11) | (g6<<5) | b5 );
    }

    void rect( int y1, int y2, int x1, int x2, pixel_t c )
    {
        for( int y=y1; y<y2; y++ )
            for( int x=x1; x<x2; x++ )
                drawPixel( y, x, c );
    }

    pixel_t getPixel(int y, int x)
    {
        if (x < 0 || x >= MAX_X || y < 0 || y >= MAX_Y)
            return wht; // outside screen = wall
        return *(pVGA + (y << YSHIFT) + x);
    }

    void dirDelta(int dir, int *dx, int *dy)
    {
        *dx = 0;
        *dy = 0;
        if (dir == DIR_UP)         { *dy = -1; }
        else if (dir == DIR_RIGHT) { *dx =  1; }
        else if (dir == DIR_DOWN)  { *dy =  1; }
        else if (dir == DIR_LEFT)  { *dx = -1; }
    }

    /* Check if a direction is safe 1 AND 2 pixels ahead from (x,y) */
    int directionIsSafe(int dir, int x, int y)
    {
        int dx, dy;
        dirDelta(dir, &dx, &dy);

        for (int step = 1; step <= 2; step++) {
            int nx = x + step * dx;
            int ny = y + step * dy;
            if (getPixel(ny, nx) != blk) {
                return 0; // blocked
            }
        }
        return 1; // both 1 and 2 ahead are black
    }

    /* MTIME HELPERS + DELAY() USING MTIME */

    /* READ 64-bit mtime */
    uint64_t read_mtime(void){

        uint32_t hi1, lo, hi2;
        do{
            hi1 = MTIME_HIGH;
            lo  = MTIME_LOW;
            hi2 = MTIME_HIGH;
        } while(hi1 != hi2);

        return (((uint64_t)hi1)<<32) | lo;
    }

    /* WRITE 64-bit mtimecmp */
    void write_mtimecmp(uint64_t value){

        MTIMECMP_HIGH = (uint32_t)(value >> 32);
        MTIMECMP_LOW  = (uint32_t)(value & 0xFFFFFFFFu);
    }

    /* Delay that waits based on mtime */
    void delay(int N){

        uint64_t start = read_mtime();
        uint64_t wait  = (uint64_t)N;

        while (read_mtime() - start < wait){
            // do nothing
        }
    }

    /* SPEED CONTROL FROM SWITCHES */
    uint32_t compute_period_from_switches(void){

        volatile int *SW_ptr = (int *) SW_BASE;
        int sw = *SW_ptr & 0x3FF; // only 10 switches, read them
        if (sw <= 0) sw = 1; // avoid zero speed
        return BASE_TICKS * (uint32_t)sw;
    }

    /* GLOBAL GAME STATE SHARED BETWEEN MAIN AND ISRs */

    const pixel_t HUMAN_COLOUR = blu;
    const pixel_t ROBOT_COLOUR = red;

    /* Starting Positions */
    int HUMAN_START_X;
    int ROBOT_START_X;
    int START_Y;

    /* Positions, directions, and alive flags */
    volatile int human_x, human_y, human_dir, human_alive;
    volatile int robot_x, robot_y, robot_dir, robot_alive;

    /* Round Status:
        round_done = 0 while playing, 1 after crash
        round_winner = 0 for tie, 1 for human, 2 for robot
    */

    volatile int round_done   = 0;
    volatile int round_winner = 0;

    /* Pending turn remembered by KEY-ISR
        0 = no turn
        -1 = turn left (KEY1)
        +1 = turn right (KEY0)
    */

    volatile int pending_turn = 0;

    /* PENDING TURN DEBUGGING LED */

    void pending_led(void){
        volatile int* LED_ptr = (int *) LED_BASE;
        int leds = *LED_ptr & ~0x3; // keep all but first 2 LEDs
        if (pending_turn == -1) leds |= 0x1; // turn on
        else if (pending_turn == 1) leds |= 0x2;
        *LED_ptr = leds;
    }


    /* FIXED: must read mcause, not mstatus */
    static inline uint32_t read_csr_mcause(void){

        uint32_t x;
        __asm__ volatile ("csrr %0, mcause" : "=r" (x));
        return x;
    }

    /* FIXED: sets trap handler address in mtvec */
    static inline void write_csr_mtvec(void *handler){

        uintptr_t addr = (uintptr_t)handler;
        __asm__ volatile ("csrw mtvec, %0" : : "r" (addr));
    }

    /* Prepares RISC-V interrupts in CSR */

    static inline void enable_interrupts(void){
        // Enable machine timer (IRQ 7) and KEY port (IRQ 18)
        uint32_t mask = (1u << 7) | (1u << 18);
        __asm__ volatile ("csrs mie, %0" :: "r"(mask));

        // Global machine interrupt enable (mstatus.mie = 1)
        __asm__ volatile ("csrs mstatus, %0" :: "r"(0x8));
    }


    /* KEY Interrupt handler
        - triggered on KEY edges
        - set/clears pending_turn
        - updates LEDR[1:0]
    */

    void key_isr(void){

        volatile int *KEY_data_ptr = (int * ) KEY_BASE;
        volatile int *KEY_edge_ptr = (int *)(KEY_BASE + 0xC);

        volatile int dummy = *KEY_data_ptr; 

        int edges = *KEY_edge_ptr; // clears the edge capture bits

        *KEY_edge_ptr = 0x3; // clear edges correctly

        if (edges & 0x1){ // if KEY0 edge
            pending_turn = (pending_turn == +1 ? 0 : +1); // right turn 
        }
        if (edges & 0x2){ // if KEY1 edge
            pending_turn = (pending_turn == -1 ? 0 : -1); // left turn
        }

        pending_led();
    }


    /* TIMER ISR
        - Applies pending turn once
        - Updates robot direction
        - Moves both players
        - Detects crashes and sets round_done + round_winner
        - Reschedules next timer interrupt using SW-based speed
    */

    void timer_isr(void){

        if(!round_done && human_alive && robot_alive){

            /* FIXED: only turn if pending_turn is exactly -1 or +1 */
            if (pending_turn == -1){
                human_dir = (human_dir + 3) % 4; // turn left
            }
            else if (pending_turn == +1){
                human_dir = (human_dir + 1) % 4; // turn right
            }

            pending_turn = 0;

            pending_led();

            int forward_safe = directionIsSafe(robot_dir, robot_x, robot_y);
            if (!forward_safe) {
                int left_dir  = (robot_dir + 3) % 4;
                int right_dir = (robot_dir + 1) % 4;

                if (directionIsSafe(left_dir, robot_x, robot_y)) {
                    robot_dir = left_dir;
                }

                else if (directionIsSafe(right_dir, robot_x, robot_y)) {
                    robot_dir = right_dir;
                }
            }

            int new_hx = human_x, new_hy = human_y;

            if(human_dir == DIR_UP)      new_hy--;
            else if(human_dir == DIR_RIGHT) new_hx++;
            else if(human_dir == DIR_DOWN)  new_hy++;
            else if(human_dir == DIR_LEFT)  new_hx--;

            int new_rx = robot_x, new_ry = robot_y;

            if(robot_dir == DIR_UP)        new_ry--;
            else if(robot_dir == DIR_RIGHT) new_rx++;
            else if(robot_dir == DIR_DOWN)  new_ry++;
            else if(robot_dir == DIR_LEFT)  new_rx--;

            /* HEAD ON COLLISIONS */
            int head_on = (new_hx == new_rx) && (new_hy == new_ry);

            int cross_over = (new_hx == robot_x && new_hy == robot_y) &&
                            (new_rx == human_x && new_ry == human_y);

            if(head_on || cross_over){
                human_alive  = 0;
                robot_alive  = 0;
                round_done   = 1;
                round_winner = 0; // tie
            }

            else{
                pixel_t h_target = getPixel(new_hy, new_hx);
                if (h_target != blk){
                    human_alive  = 0;
                    round_done   = 1;
                    round_winner = 2; // robot wins
                }
                else{
                    human_x = new_hx;
                    human_y = new_hy;
                    drawPixel(human_y, human_x, HUMAN_COLOUR);
                }
            }

            if(!round_done){
                pixel_t r_target = getPixel(new_ry, new_rx);
                if(r_target != blk){
                    robot_alive  = 0;
                    round_done   = 1;
                    round_winner = 1; // human wins
                }
                else{
                    robot_x = new_rx;
                    robot_y = new_ry;
                    drawPixel(robot_y, robot_x, ROBOT_COLOUR);
                }
            }
        }

        /* RESCHEDULE TIMER for next tick */
        uint64_t now    = read_mtime();
        uint32_t period = compute_period_from_switches();
        write_mtimecmp(now + period);
    }

    /* MAIN TRAP HANDLER
        - decides whether it was timer or KEY
        - bit 31 = 1 for interrupt
        - lower bits = cause number (timer = 7, KEY = 18 on our system)
    */

    void __attribute__((interrupt)) trap_handler(void){

        uint32_t cause = read_csr_mcause();
        int is_interrupt = (cause >> 31) & 0x1;
        uint32_t code    = cause & 0x7FFFFFFF; // ignore top bit

        if (!is_interrupt){
            // should not happen
            return;
        }

        if(code == 7){
            // machine timer
            timer_isr();
        }
        else if(code == 18){
            // pushbutton KEY
            key_isr();
        }
    }

    /* Initialize one round of the game */

    void init_round(void){
        // stop gameplay while we reset things
        round_done   = 1;
        round_winner = 0;
        human_alive  = 0;
        robot_alive  = 0;

        // clear any pending turn and LEDs
        pending_turn = 0;
        pending_led();

        // set starting directions
        human_dir = DIR_RIGHT;
        robot_dir = DIR_LEFT;

        // clear screen and draw white border
        rect(0, MAX_Y, 0, MAX_X, blk);              // clear screen to black
        rect(0, 1, 0, MAX_X, wht);                  // top border
        rect(MAX_Y-1, MAX_Y, 0, MAX_X, wht);        // bottom border
        rect(0, MAX_Y, 0, 1, wht);                  // left border
        rect(0, MAX_Y, MAX_X-1, MAX_X, wht);        // right border

        // obstacle in the middle
        int obs_w  = 20;
        int obs_h  = 20;
        int obs_x1 = (MAX_X/2) - obs_w/2;
        int obs_y1 = (MAX_Y/2) - obs_h/2;
        rect(obs_y1, obs_y1 + obs_h, obs_x1, obs_x1 + obs_w, wht);

        // set starting positions
        human_x = HUMAN_START_X;
        human_y = START_Y;
        robot_x = ROBOT_START_X;
        robot_y = START_Y;

        drawPixel(human_y, human_x, HUMAN_COLOUR);
        drawPixel(robot_y, robot_x, ROBOT_COLOUR);

        // NOW the round is ready — allow gameplay again
        human_alive = 1;
        robot_alive = 1;
        round_done  = 0;

        // schedule first timer tick for this round
        uint64_t now    = read_mtime();
        uint32_t period = compute_period_from_switches();
        write_mtimecmp(now + period);
    }

    /* Set up KEY interrupts  */

    void init_interrupts(void){

        write_csr_mtvec(trap_handler);

        // CONFIGURE KEY HARDWARE
        volatile int *KEY_mask_ptr = (int *)(KEY_BASE + 0x8); // interrupt, mask offset
        volatile int *KEY_edge_ptr = (int *)(KEY_BASE + 0xC); // edge capture offset

        *KEY_edge_ptr = 0x3; // clear any prior edge captures by writing 1s
        *KEY_mask_ptr = 0x3; // enable interrupts for KEY0 and KEY1

        enable_interrupts();
    }

    /* MAIN */

    int main(){

        printf( "start\n" );

        volatile int *HEX_ptr = (int *) HEX3_HEX0_BASE;

        /* HEX DISPLAY */
        int HEX_MAP[10] = {
        0xBF, // 0  
        0x86, // 1  
        0xDB, // 2  
        0xCF, // 3  
        0xE6, // 4  
        0xED, // 5  
        0xFD, // 6  
        0x87, // 7  
        0xFF, // 8  
        0xEF  // 9  
        };

        int humanScore = 0;
        int robotScore = 0;

        /* Show 0–0 at the start */
        int hex_value = (HEX_MAP[robotScore] << 8) | HEX_MAP[humanScore];
        *HEX_ptr = hex_value;

        /* STARTING POSITIONS */
        HUMAN_START_X = MAX_X/3;
        ROBOT_START_X = 2*MAX_X/3;
        START_Y       = MAX_Y/2;

        init_interrupts();

        while (humanScore < 9 && robotScore < 9){

            // start new round
            init_round();

            // useless loop
            while(!round_done){
                // do nothing useful
                delay(50000);
            }

            if(round_winner == 1){
                humanScore++;
            }
            else if(round_winner == 2){
                robotScore++;
            }

            // update HEX display
            hex_value = (HEX_MAP[robotScore] << 8) | HEX_MAP[humanScore];
            *HEX_ptr = hex_value;

            delay(5000000);
        }

        // GAME OVER

        if(humanScore == 9){
            rect(0, MAX_Y, 0, MAX_X, HUMAN_COLOUR);
        }
        else{
            rect(0, MAX_Y, 0, MAX_X, ROBOT_COLOUR);
        }

        while(1){
            // do nothing
        }

        return 0;
    }

