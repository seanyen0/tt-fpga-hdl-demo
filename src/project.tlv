\m5_TLV_version 1d: tl-x.org
\m5
   use(m5-1.0)
   
   
   // ########################################################
   // #                                                      #
   // #  Empty template for Tiny Tapeout Makerchip Projects  #
   // #                                                      #
   // ########################################################
   
   // ========
   // Settings
   // ========
   
   //-------------------------------------------------------
   // Build Target Configuration
   //
   var(my_design, tt_um_example)   /// The name of your top-level TT module, to match your info.yml.
   var(target, ASIC)   /// Note, the FPGA CI flow will set this to FPGA.
   //-------------------------------------------------------
   
   var(in_fpga, 1)   /// 1 to include the demo board. (Note: Logic will be under /fpga_pins/fpga.)
   var(debounce_inputs, m5_if_defined_as(MAKERCHIP, 1, 0, 1))         /// 1: Provide synchronization and debouncing on all input signals.
                                   /// 0: Don't provide synchronization and debouncing.
                                   /// m5_if_defined_as(MAKERCHIP, 1, 0, 1): Debounce unless in Makerchip.
   //user variables
   define_hier(DEPTH, 8) // max bits in correct sequence. Needs to be even. 
                          // _hier = there are multiple linked variables. _INDEX_MAX is log2 of the game counter max count. _CNT is the value of max count.
   define_hier(CLKS_PER_ADV,20000000) // subdivide system clock into human viewable clock. Eventually 20M for 1s period
   var(clks_per_led_off, 3000000) // # of clocks for LED to flash off (to delimit count). Must be < clks_per_adv
   
   
   // ======================
   // Computed From Settings
   // ======================
   
   // If debouncing, a user's module is within a wrapper, so it has a different name.
   var(user_module_name, m5_if(m5_debounce_inputs, my_design, m5_my_design))
   var(debounce_cnt, m5_if_defined_as(MAKERCHIP, 1, 8'h03, 8'hff))

\SV
   // Include Tiny Tapeout Lab.
   m4_include_lib(['https:/']['/raw.githubusercontent.com/os-fpga/Virtual-FPGA-Lab/35e36bd144fddd75495d4cbc01c4fc50ac5bde6f/tlv_lib/tiny_tapeout_lib.tlv'])


\TLV my_design()
   
   
   
   // ==================
   // |                |
   // | YOUR CODE HERE |
   
   |simon
      //COUNTER VARIABLES:
      // $game_cnt: counting signal for advancing the "correct color". Max value = m5_DEPTH_CNT
      // $game_stg: stage-counting signal for max(game_cnt). Increment one more stage if user is successful  
      // $game_rnd: "game round": controls speed of clock
      
      //STATE VARIABLE: $state_guess: 
      // 0: playback of correct sequence to be imitated.
      // 1: user guesses sequence
      
      //USER VARIABLES:
      // $user_guess: user input guess
      @-1
         $reset_in = *reset || *ui_in[7] || >>3$lose_game && >>2$user_button_press;
         
      @1   
         //native clock count. Feeds into subdividing into human-respondable clock $game_cnt.
         $ii[m5_CLKS_PER_ADV_INDEX_MAX:0] = $reset || >>1$ii==m5_calc(m5_CLKS_PER_ADV_CNT-1) || >>1$win_stg
                    ? 0 :
                    >>1$ii + 1; // $ii will loop back to zero when it reaches m5_DEPTH_INDEX_MAX bits.
         
         // wait by one second after reset
         $reset_counter[m5_CLKS_PER_ADV_INDEX_MAX:0] = $reset_in
                    ? 0 :
                    >>1$reset_counter == m5_calc(m5_CLKS_PER_ADV_CNT-1)
                    ? >>1$reset_counter :
                    >>1$reset_counter + 1; // $ii will loop back to zero when it reaches m5_DEPTH_INDEX_MAX bits.
         $reset = ($reset_counter == m5_calc(m5_CLKS_PER_ADV_CNT-1)) && (>>1$reset_counter == m5_calc(m5_CLKS_PER_ADV_CNT-2));
         
         
         // game state: 1 = user is guessing. 0 = something else
         $state_guess = $reset || >>1$win_stg
                       ? 1'b0 :
                       $advance_game_cnt && ( >>1$game_cnt == >>1$game_stg-1 )
                       ? 1'b1:
                       >>1$state_guess;
         
         
         $advance_game_cnt = ( ! >>1$state_guess && ($ii == m5_calc(m5_CLKS_PER_ADV_CNT-1)) )
                            || ( >>1$state_guess &&  >>1$user_button_press);
         
         // game counter
         $game_cnt[m5_DEPTH_INDEX_MAX:0] =
                    $reset || >>1$win_stg || $state_guess && ! >>1$state_guess //human-respondable game clock/count. No "-1" because you want game_cnt to be 2x the depth of the data (due to two bits per count)
                    ? 0 :
                    (! $state_guess && $advance_game_cnt && ( >>1$game_cnt < >>1$game_stg ) ) || ( >>1$state_guess && >>1$correct_guess )
                    ? >>1$game_cnt + 1 :
                    >>1$game_cnt;
         
         $game_stg[m5_DEPTH_INDEX_MAX:0] = $reset || >>1$game_stg == m5_calc(m5_DEPTH_CNT/2) //stage-counting signal for max(game_cnt). Increment one more stage if user is successful  
                    ? 1 :
                    $win_stg == 1 && >>1$game_stg != m5_calc(m5_DEPTH_CNT/2)
                    ? >>1$game_stg + 1 :
                    >>1$game_stg;
         
         
         $correct_seq[m5_DEPTH_MAX:0] = m5_DEPTH_CNT'hD6;
         $color[1:0] = $correct_seq[ ($game_cnt + $game_cnt) +: 2];
         
         // USER INPUT PHASE (should include $state_guess in conditionals!)
         $user_input[3:0] = *ui_in[3:0]; //"continuous" intake of user buttons
         $user_button_press = ( >>1$user_input == 4'b0 ) && ( $user_input != 4'b0 ) && >>1$state_guess; // User must release previous button before pressing the next button
         $user_guess[1:0] = $user_button_press && $user_input == 4'b0001
                       ? 2'b00 :
                       $user_button_press && $user_input == 4'b0010
                       ? 2'b01 :
                       $user_button_press && $user_input == 4'b0100
                       ? 2'b10 :
                       $user_button_press && $user_input == 4'b1000
                       ? 2'b11 :
                       >>1$user_guess;
         
         // Define win/loss/correct/etc.
         $correct_guess = $user_button_press && ($user_guess == $color);
         $lose_game = $user_button_press & ! $correct_guess
                      ? 1 :
                      $reset
                      ? 0 :
                      >>1$lose_game;
         $win_stg_in = (>>1$user_input == 4'b0) && >>1$state_guess && ( >>1$game_cnt == >>1$game_stg ); //WAS: ! >>1$lose_game && ( >>1$game_cnt == >>1$game_stg )
         $win_stg_counter[m5_CLKS_PER_ADV_INDEX_MAX:0] = $win_stg_in && ! >>1$win_stg_in
                    ? 0 :
                    >>1$win_stg_counter == m5_calc(m5_CLKS_PER_ADV_CNT-1)
                    ? >>1$win_stg_counter :
                    >>1$win_stg_counter + 1;
         $win_stg = ($win_stg_counter == m5_calc(m5_CLKS_PER_ADV_CNT-1)) && (>>1$win_stg_counter == m5_calc(m5_CLKS_PER_ADV_CNT-2));
         
         // Display the sequence to the user: flash off before turning on
         $sseg_out[7:0] = (! $state_guess && ( $ii > m5_CLKS_PER_ADV_MAX - m5_clks_per_led_off  || $advance_game_cnt ) ) || $reset
                     ? 8'b10000000 : //80, just the dot
                     $lose_game
                     ? 8'b00111000 : //"L", 0x38
                     ( ! $state_guess && $color == 0 && ($game_cnt < $game_stg) ) || ( $state_guess && $user_input != 0 && $user_guess == 0 )
                     ? 8'b00111111: //3f
                     ( ! $state_guess && $color == 1 && ($game_cnt < $game_stg) ) || ( $state_guess && $user_input != 0 && $user_guess == 1 )
                     ? 8'b00000110: //06
                     ( ! $state_guess && $color == 2 && ($game_cnt < $game_stg) ) || ( $state_guess && $user_input != 0 && $user_guess == 2 )
                     ? 8'b01011011: //5b
                     ( ! $state_guess && $color == 3 && ($game_cnt < $game_stg) ) || ( $state_guess && $user_input != 0 && $user_guess == 3 )
                     ? 8'b01001111 : //4f
                     8'b01000000;
         
         // m5+sseg_decoder($digits_out, $digits_in)
         *uo_out = $sseg_out;
   // |                |
   // ==================
   
   // Note that pipesignals assigned here can be found under /fpga_pins/fpga.
   
   
   
   
   // Connect Tiny Tapeout outputs. Note that uio_ outputs are not available in the Tiny-Tapeout-3-based FPGA boards.
   //*uo_out = 8'b0;
   m5_if_neq(m5_target, FPGA, ['*uio_out = 8'b0;'])
   m5_if_neq(m5_target, FPGA, ['*uio_oe = 8'b0;'])

// Set up the Tiny Tapeout lab environment.
\TLV tt_lab()
   // Connect Tiny Tapeout I/Os to Virtual FPGA Lab.
   m5+tt_connections()
   // Instantiate the Virtual FPGA Lab.
   m5+board(/top, /fpga, 7, $, , my_design)
   // Label the switch inputs [0..7] (1..8 on the physical switch panel) (top-to-bottom).
   m5+tt_input_labels_viz(['"UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED", "UNUSED"'])

\SV

// ================================================
// A simple Makerchip Verilog test bench driving random stimulus.
// Modify the module contents to your needs.
// ================================================

module top(input logic clk, input logic reset, input logic [31:0] cyc_cnt, output logic passed, output logic failed);
   // Tiny tapeout I/O signals.
   logic [7:0] ui_in, uo_out;
   m5_if_neq(m5_target, FPGA, ['logic [7:0] uio_in, uio_out, uio_oe;'])
   logic [31:0] r;  // a random value
   always @(posedge clk) r <= m5_if_defined_as(MAKERCHIP, 1, ['$urandom()'], ['0']);
   //assign ui_in = r[7:0]; //comment this out if using specific inputs (5 lines below)
   m5_if_neq(m5_target, FPGA, ['assign uio_in = 8'b0;'])
   logic ena = 1'b0;
   logic rst_n = ! reset;
   
   
   // Or, to provide specific inputs at specific times (as for lab C-TB) ...
   // BE SURE TO COMMENT THE ASSIGNMENT OF INPUTS ABOVE. "assign ui_in = r[7:0];"
   // BE SURE TO DRIVE THESE ON THE B-PHASE OF THE CLOCK (ODD STEPS).
   // Driving on the rising clock edge creates a race with the clock that has unpredictable simulation behavior.
   initial begin
      #1  // Drive inputs on the B-phase.
         ui_in = 8'h0;
      #100// Step #_/2 cycles, past reset.
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      
      #80
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      
      
      #40
         ui_in = 8'h04;
      #2
         ui_in = 8'h00;
      #2
         ui_in = 8'h02;
      #2
         ui_in = 8'h00;
      #2
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      
      
      #120
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h08;
      #8
         ui_in = 8'h00;
      
      
      #80
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h08;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      
      
      
      #20
         ui_in = 8'h08;
      #8
         ui_in = 8'h00;
      
      #100// Step #_/2 cycles, past reset.
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      
      #40
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      
      
      #40
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      
      
      #80
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h08;
      #8
         ui_in = 8'h00;
      
      
      #80
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h02;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h08;
      #8
         ui_in = 8'h00;
      #20
         ui_in = 8'h04;
      #8
         ui_in = 8'h00;
      
      // ...etc.
   end

   // Instantiate the Tiny Tapeout module.
   m5_user_module_name tt(.*);
   
   assign passed = top.cyc_cnt > 400;
   assign failed = 1'b0;
endmodule


// Provide a wrapper module to debounce input signals if requested.
m5_if(m5_debounce_inputs, ['m5_tt_top(m5_my_design)'])
\SV



// =======================
// The Tiny Tapeout module
// =======================

module m5_user_module_name (
    input  wire [7:0] ui_in,    // Dedicated inputs - connected to the input switches
    output wire [7:0] uo_out,   // Dedicated outputs - connected to the 7 segment display
    m5_if_eq(m5_target, FPGA, ['/']['*'])   // The FPGA is based on TinyTapeout 3 which has no bidirectional I/Os (vs. TT6 for the ASIC).
    input  wire [7:0] uio_in,   // IOs: Bidirectional Input path
    output wire [7:0] uio_out,  // IOs: Bidirectional Output path
    output wire [7:0] uio_oe,   // IOs: Bidirectional Enable path (active high: 0=input, 1=output)
    m5_if_eq(m5_target, FPGA, ['*']['/'])
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
   wire reset = ! rst_n;

\TLV
   /* verilator lint_off UNOPTFLAT */
   m5_if(m5_in_fpga, ['m5+tt_lab()'], ['m5+my_design()'])

\SV_plus
   
   // ==========================================
   // If you are using Verilog for your design,
   // your Verilog logic goes here.
   // Note, output assignments are in my_design.
   // ==========================================

\SV
endmodule
