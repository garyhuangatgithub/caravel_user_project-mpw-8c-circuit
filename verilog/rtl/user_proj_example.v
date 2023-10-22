// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

// Source http://github.com/efabless/caravel_user_project mpw-8c

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

// Generalized Reed-Solomon Error Correction Code
// at Quadratic Order in Binary Characteristics
// with 2 Bit Error Correction Capability
// A verilog implementation of the c language Art of ECC bch_bm.c
// Source http://the-art-of-ecc.com/3_Cyclic_BCH/bch_bm.c

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [`MPRJ_IO_PADS-1:0] eccOut;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // IO
    assign io_out = eccOut;
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    assign la_data_out = {{(127-`MPRJ_IO_PADS){1'b0}}, eccOut};
    // Assuming LA probes [63:32] are for controlling the count register
    assign la_write = ~la_oenb[63:32] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    eccqobicIO #() eccqobicIO
    (
        .clk(clk),
        .reset(rst),
        .codecOut(eccOut[21:7]),
        .complete(eccOut[5:5]),
        .la_write(la_write),
        .la_input(la_data_in[63:32])
    );

endmodule

module eccqobicIO
(
  input wire clk,             // Clock
  input wire reset,           // Reset to a known good state
  output [14:0] codecOut,     // Encoded codeword or decoded message
  output complete,            // Complete
  input [31:0] la_write,      // Logic analyzer write assert
  input [31:0] la_input       // Logic analyzer with input message, start, codec mode
);

  // ECC Codec
  eccqobic codecCtrl
  (
    .clk (codecCtrl$clk),
    .rst (codecCtrl$rst),
    .codecMode (codecCtrl$codecMode),
    .start (codecCtrl$start),
    .codecIn (codecCtrl$codecIn),
    .complete (codecCtrl$complete),
    .codecOut (codecCtrl$codecOut)
  );

  wire [0:0] codecCtrl$clk;
  wire [0:0] codecCtrl$rst;
  wire [0:0] codecCtrl$codecMode;
  wire [0:0] codecCtrl$start;
  wire [14:0] codecCtrl$codecIn;
  wire [0:0] codecCtrl$complete;
  wire [14:0] codecCtrl$codecOut;

  reg [14:0] codecInReg = 15'b000000000000000;
  reg [0:0] startReg = 1'b0;
  reg [0:0] codecModeReg = 1'b0;
  reg [14:0] codecOutReg = 15'b000000000000000;
  reg [0:0] completeReg = 1'b0;

  assign codecOut = codecOutReg;
  assign complete = completeReg;

  always @ (posedge clk) begin
    if (|la_write) begin
      if (la_write[14:0]) begin
        codecInReg <= la_write[14:0] & la_input[14:0];
      end
      if (la_write[15:15]) begin
        startReg <= la_write[15:15] & la_input[15:15];
      end
      if (la_write[16:16]) begin
        codecModeReg <= la_write[16:16] & la_input[16:16];
      end
    end else begin
      codecOutReg <= codecCtrl$codecOut;
      completeReg <= codecCtrl$complete;
    end
  end

  assign codecCtrl$clk = clk;
  assign codecCtrl$rst = reset;
  assign codecCtrl$codecIn = codecInReg;
  assign codecCtrl$start = startReg;   
  assign codecCtrl$codecMode = codecModeReg;

endmodule // eccqobicIO

module eccqobic
(
  input wire clk,             // Clock
  input wire rst,             // Reset to a known good state
  input [14:0] codecIn,       // Input message or input code
  input wire start,           // Start
  input wire codecMode,       // Codec encode decode mode
  output [14:0] codecOut,     // Encoded codeword or decoded message
  output complete             // Complete
);

  // ECC Encoder
  wire [0:0] encCtrl$clk;
  wire [0:0] encCtrl$reset;
  wire [0:0] encCtrl$start;
  wire [14:0] encCtrl$message;
  wire [0:0] encCtrl$complete;
  wire [14:0] encCtrl$recd;

  EccEncodeControl encCtrl
  (
    .clk (encCtrl$clk),
    .reset (encCtrl$reset),
    .start (encCtrl$start),
    .message (encCtrl$message),
    .complete (encCtrl$complete),
    .recd (encCtrl$recd)
  );

  assign encCtrl$clk = clk;
  assign encCtrl$reset = rst;
  assign encCtrl$message = codecIn;

  // ECC Decoder
  wire decCtrl$clk;
  wire decCtrl$reset;
  wire decCtrl$start;
  wire [14:0] decCtrl$recd_;
  wire decCtrl$complete;
  wire [14:0] decCtrl$message_;

  EccDecodeControl decCtrl
  (
    .clk (decCtrl$clk),
    .reset (decCtrl$reset),
    .start (decCtrl$start),
    .recd_ (decCtrl$recd_),
    .complete (decCtrl$complete),
    .message_ (decCtrl$message_)
  );

  assign decCtrl$clk = clk;
  assign decCtrl$reset = rst;
  assign decCtrl$recd_ = codecIn;

  // Codec Mode Decoder
  wire [0:0] driverStart$clk;
  wire [0:0] driverStart$in;
  wire [0:0] driverStart$sel;
  wire [0:0] driverStart$out_$000;
  wire [0:0] driverStart$out_$001;

  Driver_0x462543a0 driverStart
  (
    .clk      ( driverStart$clk ),
    .in       ( driverStart$in ),
    .sel      ( driverStart$sel ),
    .out_$000 ( driverStart$out_$000 ),
    .out_$001 ( driverStart$out_$001 )
  );

  assign driverStart$clk = clk;
  assign driverStart$in = start;
  assign driverStart$sel = codecMode;
  assign encCtrl$start = driverStart$out_$000;
  assign decCtrl$start = driverStart$out_$001;

  // Codec Mode Multiplexer
  wire [0:0] muxComplete$clk;
  wire [0:0] muxComplete$in_$000;
  wire [0:0] muxComplete$in_$001;
  wire [0:0] muxComplete$sel;
  wire [0:0] muxComplete$out;

  Mux_0xa543df60 muxComplete
  (
    .clk     ( muxComplete$clk ),
    .in_$000 ( muxComplete$in_$000 ),
    .in_$001 ( muxComplete$in_$001 ),
    .sel     ( muxComplete$sel ),
    .out     ( muxComplete$out )
  );

  assign muxComplete$clk = clk;
  assign muxComplete$sel = codecMode;
  assign muxComplete$in_$000 = encCtrl$complete;
  assign muxComplete$in_$001 = decCtrl$complete;
  assign complete = muxComplete$out;

  wire [0:0] muxOut$clk;
  wire [14:0] muxOut$in_$000;
  wire [14:0] muxOut$in_$001;
  wire [0:0] muxOut$sel;
  wire [14:0] muxOut$out;

  Mux_0xa543df63 muxOut
  (
    .clk     ( muxOut$clk ),
    .in_$000 ( muxOut$in_$000 ),
    .in_$001 ( muxOut$in_$001 ),
    .sel     ( muxOut$sel ),
    .out     ( muxOut$out )
  );

  assign muxOut$clk = clk;
  assign muxOut$sel = codecMode;
  assign muxOut$in_$000 = encCtrl$recd;
  assign muxOut$in_$001 = decCtrl$message_;
  assign codecOut = muxOut$out;
   
endmodule  // eccqobic

//-----------------------------------------------------------------------------
// EccEncodeControl
// ECC Encoder Control Unit
// Interface Protocol
// The reset signal takes place in the first clocking to initialize the system
// to a known good state
// reg clk;
// reg rst;
// reg start;
// reg [14:0] codecIn;
// wire complete;
// wire [14:0] codecOut;
// eccqobic eccencode(clk, rst, start, codecIn, complete, codecOut);
// initial begin
//   clk=0; rst=0; mode=0; start=0;
//   #1 #1 rst=1; #1 #1 rst=0;  // Reset system to a known good state
//   // Message 7b'1001011
//   // Encoded codeword 15'b100101101010111
//   codecIn={8'b00000000,7'b1001011}; start=1;
//   #1 #1 start=0;  // Debounce start signal
//   #1 #1 #1 #1 #1
//   $display("Message = %b", codecIn[6:0]);
//   $display("Code encoded = %b", codecOut);
//   $display("Encoding completed");
// end
//-----------------------------------------------------------------------------
module EccEncodeControl
(
  input wire clk,             // Clock
  input wire reset,           // Reset to a known good state
  input wire start,           // Kick off the encoding
  input wire [14:0] message,  // User message
  output wire complete,       // Encoding completes
  output wire [14:0] recd     // Encoded codeword
);

  // Local parameters declarations
  localparam MESSAGE_LENGTH = 7;
  localparam CODE_LENGTH = 15;
  localparam GENERATOR_LENGTH = CODE_LENGTH - MESSAGE_LENGTH;
  localparam MAX_MESSAGE_INDEX = MESSAGE_LENGTH - 1;
  localparam MAX_CODE_INDEX = CODE_LENGTH - 1;
  localparam MAX_GENERATOR_INDEX = CODE_LENGTH - MESSAGE_LENGTH - 1;

  // ECC code generator
  wire [15:0] gen[0:CODE_LENGTH - MESSAGE_LENGTH];

  assign gen[0] = 1;
  assign gen[1] = 0;
  assign gen[2] = 0;
  assign gen[3] = 0;
  assign gen[4] = 1;
  assign gen[5] = 0;
  assign gen[6] = 1;
  assign gen[7] = 1;
  assign gen[8] = 1;

  // State register
  wire [0:0] state$clk;
  wire [0:0] state$reset;
  wire [1:0] state$out;
  reg [1:0] state$in_;

  RegRst_0xd3f9810a state
  (
    .clk ( state$clk ),
    .in_ ( state$in_ ),
    .out ( state$out ),
    .reset ( state$reset )
  );

  assign state$clk = clk;
  assign state$reset = reset;

  localparam STATE_IDLE = 0;
  localparam STATE_CALC = 1;
  localparam STATE_REPLIC = 2;
  localparam STATE_DONE = 3;

  reg [2:0] current_state__0;
  reg [2:0] next_state__0;
  reg [2:0] current_state__1;

  // Codeword register
  wire [0:0] recdReg$clk;
  wire [0:0] recdReg$reset;
  wire [14:0] recdReg$out;
  reg [14:0] recdReg$in_;

  RegRst_0xd3f9810f recdReg
  (
    .clk (recdReg$clk),
    .in_ (recdReg$in_),
    .out (recdReg$out),
    .reset (recdReg$reset)
  );

  assign recdReg$clk = clk;
  assign recdReg$reset = reset;
  assign recd = recdReg$out;  // Connect to interface

  // Complete register
  wire [0:0] completeReg$clk;
  wire [0:0] completeReg$reset;
  wire [0:0] completeReg$out;
  reg [0:0] completeReg$in_;

  RegRst_0xd3f98109 completeReg
  (
    .clk (completeReg$clk),
    .in_ (completeReg$in_),
    .out (completeReg$out),
    .reset (completeReg$reset)
  );

  assign completeReg$clk = clk;
  assign completeReg$reset = reset;
  assign complete = completeReg$out;  // Connect to interface

  reg [15:0] bb[0:15];
  reg [15:0] feedback;

  integer i;
  integer j;

  initial begin
    state$in_ = 2'b00;
  end

  // State transition
  always @ (posedge clk) begin
    current_state__0 = state$out;
    next_state__0 = state$out;

    case (current_state__0)
      STATE_IDLE: begin
        if (start) begin
          next_state__0 = STATE_CALC;
        end
      end

      STATE_CALC: begin
        next_state__0 = STATE_REPLIC;
      end

      STATE_REPLIC: begin
        next_state__0 = STATE_DONE;
      end

      STATE_DONE: begin
        if (reset) begin
          next_state__0 = STATE_IDLE;
        end
        else begin
          next_state__0 = STATE_DONE;
        end
      end

      default: begin
      end       
    endcase

    state$in_ = next_state__0;
  end   

  // State output
  always @ (posedge clk) begin
    current_state__1 = state$out;

    case (current_state__1)
      STATE_IDLE: begin
        completeReg$in_ = 0;
      end

      STATE_CALC: begin
        for (i = 0; i < GENERATOR_LENGTH; i++) begin
          bb[i] = 0;
        end
        for (i = MESSAGE_LENGTH - 1; i >= 0; i--) begin
          feedback = message[i] ^ bb[CODE_LENGTH - MESSAGE_LENGTH - 1];
          if (feedback != 0) begin
            for (j = CODE_LENGTH - MESSAGE_LENGTH - 1; j > 0; j--) begin
              if (gen[j] != 0) begin
                bb[j] = bb[j - 1] ^ feedback;
              end
              else begin
                bb[j] = bb[j - 1];
              end
            end
            bb[0] = gen[0] && feedback;
          end
          else begin
            for (j = CODE_LENGTH - MESSAGE_LENGTH - 1; j > 0; j--) begin
              bb[j] = bb[j - 1];
            end
            bb[0] = 0;
          end
        end
      end

      STATE_REPLIC: begin
        recdReg$in_ = 0;
        for (i = 0; i < CODE_LENGTH - MESSAGE_LENGTH; i++) begin
          if (bb[i] & 1) begin
            recdReg$in_ = recdReg$in_ | (15'b000000000000001 << i);
          end
        end
        for (i = 0; i < MESSAGE_LENGTH; i++) begin
          if (message[i] & 1) begin
            recdReg$in_ = recdReg$in_ | (15'b000000000000001 << (i + CODE_LENGTH - MESSAGE_LENGTH));
          end
        end
      end

      STATE_DONE: begin
        completeReg$in_ = 1;
      end

      default: begin
      end       
    endcase
  end

endmodule  //EccEncodeControl

//-----------------------------------------------------------------------------
// EccDecodeControl
// ECC Decoder Control Unit
// Interface Protocol
// The reset signal takes place in the first clocking to initialize the system
// to a known good state
// reg clk;
// reg rst;
// reg start;
// reg [14:0] codecIn;
// wire complete;
// wire [14:0] codecOut;
// eccqobic eccdecode(clk, rst, start, codecIn, complete, codecOut);
// initial begin
//   clk=0; rst=0; mode=0; start=0;
//   #1 #1 rst=1; #1 #1 rst=0;  // Reset system to a known good state
//   // Error-free codeword 15'b100101101010111
//   // Inject 2 errors at location 3 and 10
//   // Expected corrected data 7'b1001011
//   codecIn=15'b100111101011111; mode=1; start=1;
//   #1 #1 start=0;  // Debounce start signal
//   #1 #1 #1 #1 #1 #1 #1 #1 #1 #1
//   #1 #1 #1 #1 #1 #1 #1 #1 #1 #1
//   #1 #1 #1 #1 #1 #1 #1 #1 #1 #1
//   $display("Code = %b", codecIn);
//   $display("Message decoded = %b", codecOut[6:0]);
//   $display("Decoding completed");
// end
//-----------------------------------------------------------------------------
module EccDecodeControl
(
  input wire clk,              // Clock
  input wire reset,            // Reset to a known good state
  input wire start,            // Kick off the decoding
  input wire [14:0] recd_,     // Input code
  output wire complete,        // Decoding completes
  output wire [14:0] message_  // Decoded message
);

  // Galois field arithmetic look-up table for computation acceleration
  wire [15:0] alpha_to[0:15];
  wire [15:0] index_of[0:15];

  assign alpha_to[0] = 1;    // 0001 Polynomial 0(x**3) + 0(x**2) + 0(x**1) + 1
  assign alpha_to[1] = 2;    // 0010 Polynomial 0(x**3) + 0(x**2) + 1(x**1) + 0
  assign alpha_to[2] = 4;    // 0100 Polynomial 0(x**3) + 1(x**2) + 0(x**1) + 0
  assign alpha_to[3] = 8;    // 1000 Polynomial 1(x**3) + 0(x**2) + 0(x**1) + 0
  assign alpha_to[4] = 3;    // 0011
  assign alpha_to[5] = 6;    // 0110
  assign alpha_to[6] = 12;   // 1100
  assign alpha_to[7] = 11;   // 1011
  assign alpha_to[8] = 5;    // 0101
  assign alpha_to[9] = 10;   // 1001
  assign alpha_to[10] = 7;   // 0111
  assign alpha_to[11] = 14;  // 1110
  assign alpha_to[12] = 15;  // 1111
  assign alpha_to[13] = 13;  // 1101
  assign alpha_to[14] = 9;   // 1001
  assign alpha_to[15] = 0;

  assign index_of[0] = -1;   // null or zero
  assign index_of[1] = 0;    // alpha**0
  assign index_of[2] = 1;    // alpha**1
  assign index_of[3] = 4;    // alpha**2
  assign index_of[4] = 2;    // alpha**3
  assign index_of[5] = 8;    // alpha**4
  assign index_of[6] = 5;    // alpha**5
  assign index_of[7] = 10;   // alpha**6
  assign index_of[8] = 3;    // alpha**7
  assign index_of[9] = 14;   // alpha**8
  assign index_of[10] = 9;   // alpha**9
  assign index_of[11] = 7;   // alpha**10
  assign index_of[12] = 6;   // alpha**11
  assign index_of[13] = 13;  // alpha**12
  assign index_of[14] = 11;  // alpha**13
  assign index_of[15] = 12;  // alpha**14

  // Local parameters
  localparam NUM_ERROR_CORRECT = 2;
  localparam TWO_T = 2 * NUM_ERROR_CORRECT;
  localparam FINITE_FILED_EXTEND_POWER = 4;
  localparam FINITE_FILED_ORDER = 2 ** 4;
  localparam n = 16'b0000000000001111; // FINITE_FIELD_ORDER - 1
  localparam CODE_LENGTH = 15;
  localparam POLY_LENGTH = 15;
  localparam MAX_LENGTH = 4;  // Maximum degree of error location polynomial
  localparam MAX_CAL_POLY_ORDER = 2;  // Maximum Calculated Location Polynomial

  localparam STATE_IDLE = 0;
  localparam STATE_SYNDROME = 1;
  localparam STATE_CAL_POLY = 2;
  localparam STATE_CHIEN = 3;
  localparam STATE_CORRECT = 4;
  localparam STATE_DONE = 5;

  reg [2:0] current_state__0;
  reg [2:0] next_state__0;
  reg [2:0] current_state__1;

  reg [15:0] recd[0:15];                // Received code, actual use recd[0:14]
  reg [15:0] s[0:4];                    // Syndrome
  reg [15:0] elp[0:MAX_LENGTH+1][0:3];  // Error Location Polynomial
  reg [5:0] root[0:TWO_T];              // Root
  reg [5:0] loc[0:TWO_T];               // Location
  reg [15:0] regA[0:MAX_LENGTH+1];      // Intermediate pasokon register in chien search
  reg [4:0] l[0:MAX_LENGTH+1];          // Length

  reg [15:0] syndrome01;
  reg [15:0] syndrome02;
  reg [15:0] syndrome03;
  reg [15:0] syndrome04;
  reg [0:0] syndWR;
  reg [4:0] syndSel;

  reg [6:0] message;  // Decoded message, use [6:0]
  assign message_ = { 8'b0000000, message };  // Connect to interface

  // Complete register
  wire [0:0] completeReg$clk;
  wire [0:0] completeReg$reset;
  wire [0:0] completeReg$out;
  reg [0:0] completeReg$in_;

  RegRst_0xd3f98109 completeReg
  (
    .clk (completeReg$clk),
    .in_ (completeReg$in_),
    .out (completeReg$out),
    .reset (completeReg$reset)
  );

  assign completeReg$clk = clk;
  assign completeReg$reset = reset;
  assign complete = completeReg$out;  // Connect to interface

  // State register
  wire [0:0] state$reset;
  wire [0:0] state$clk;
  wire [2:0] state$out;
  reg [2:0] state$in_;

  RegRst_0xd3f9810b state
  (
    .clk (state$clk),
    .in_ (state$in_),
    .out (state$out),
    .reset (state$reset)
  );

  assign state$clk = clk;
  assign state$reset = reset;

  // Berlekamp Massey algorithm
  reg [0:0] bmstart;
  wire [0:0] ctrlbm$start;
  wire [0:0] ctrlbm$clk;
  wire [0:0] ctrlbm$reset;
  wire [15:0] ctrlbm$syndrome0;
  wire [15:0] ctrlbm$syndrome1;
  wire [15:0] ctrlbm$syndrome2;
  wire [15:0] ctrlbm$syndrome3;
  wire [15:0] ctrlbm$syndrome4;
  wire [0:0] ctrlbm$complete;
  wire [0:0] ctrlbm$elpValid;
  wire [15:0] ctrlbm$elp0;
  wire [15:0] ctrlbm$elp1;
  wire [15:0] ctrlbm$elp2;

  EccDecodeControlBm ctrlbm
  (
    .clk (ctrlbm$clk),
    .reset (ctrlbm$reset),
    .start (ctrlbm$start),
    .syndrome0 (ctrlbm$syndrome0),
    .syndrome1 (ctrlbm$syndrome1),
    .syndrome2 (ctrlbm$syndrome2),
    .syndrome3 (ctrlbm$syndrome3),
    .syndrome4 (ctrlbm$syndrome4),
    .complete (ctrlbm$complete),
    .elpValid (ctrlbm$elpValid),
    .elp0 (ctrlbm$elp0),
    .elp1 (ctrlbm$elp1),
    .elp2 (ctrlbm$elp2)
  );

  assign ctrlbm$clk = clk;
  assign ctrlbm$reset = reset;
  assign ctrlbm$syndrome0 = s[0];
  assign ctrlbm$syndrome1 = s[1];
  assign ctrlbm$syndrome2 = s[2];
  assign ctrlbm$syndrome3 = s[3];
  assign ctrlbm$syndrome4 = s[4];
  assign ctrlbm$start = bmstart;

  integer i;
  integer j;
  integer u;
  integer q;
  integer count;

  integer syndromeComplete;  // Debounce STATE_SYNDROME
  integer calpolyComplete;   // Debounce STATE_CAL_POLY
  integer chienComplete;     // Debounce STATE_CHIEN
  integer correctComplete;   // Debounce STATE_CORRECT

  // Initialize state
  initial begin
    state$in_ = 0;
    completeReg$in_ = 0;
    syndromeComplete = 0;
    calpolyComplete = 0;
    chienComplete = 0;
    correctComplete = 0;
    bmstart = 0;
  end

  // State transition
  always @ (*) begin
    current_state__0 = state$out;
    next_state__0 = state$out;

    case (current_state__0)
      STATE_IDLE: begin
        if (start) begin
          for (i = 0; i < CODE_LENGTH; i++) begin
            recd[i] = (recd_ >> i) & 1;
          end
          syndromeComplete = 0;
          next_state__0 = STATE_SYNDROME;
        end
        completeReg$in_ = 0;
      end

      STATE_SYNDROME: begin
        if (syndromeComplete) begin
          calpolyComplete = 0;
          bmstart = 0;
          next_state__0 = STATE_CAL_POLY;
        end
      end

      STATE_CAL_POLY: begin
        if (calpolyComplete) begin
          chienComplete = 0;
          next_state__0 = STATE_CHIEN;
        end
      end

      STATE_CHIEN: begin
        if (chienComplete) begin
          correctComplete = 0;
          next_state__0 = STATE_CORRECT;
        end
      end

      STATE_CORRECT: begin
        if (correctComplete) begin
          message = 0;
          for (i = 8; i < CODE_LENGTH; i++) begin
            message = message + ((recd[i] & 1) << (i - 8));
          end
          next_state__0 = STATE_DONE;
        end
      end

      STATE_DONE: begin
        if (reset) begin
          next_state__0 = STATE_IDLE;
        end
        else begin
          next_state__0 = STATE_DONE;
        end
      end

      default: begin
      end       
    endcase

    state$in_ = next_state__0;
  end

  // State output
  always @ (*) begin
    current_state__1 = state$out;

    case (current_state__1)
      STATE_IDLE: begin
      end

      STATE_SYNDROME: begin
        for (i = 1; i <= TWO_T; i++) begin
          s[i] = 0;
          for (j = 0; j < POLY_LENGTH; j++) begin
            if (recd[j] != 0) begin
              s[i] = s[i] ^ alpha_to[(i * j) % POLY_LENGTH];
            end
          end
          s[i] = index_of[s[i]];
        end
        syndromeComplete = 1;
      end

      STATE_CAL_POLY: begin
        if (bmstart == 0) begin
          bmstart = 1;
        end
        if (ctrlbm$complete) begin
          elp[MAX_LENGTH+1][0] = ctrlbm$elp0;  // Replicate result ELP into array
          elp[MAX_LENGTH+1][1] = ctrlbm$elp1;
          elp[MAX_LENGTH+1][2] = ctrlbm$elp2;
          elp[MAX_LENGTH+1][3] = 0;
          bmstart = 0;
          calpolyComplete = 1;
        end
      end

      STATE_CHIEN: begin
        u = MAX_LENGTH;
        u = u + 1;
        l[u] = MAX_CAL_POLY_ORDER;
        if (l[u] <= NUM_ERROR_CORRECT) begin
          for (i = 0; i <= MAX_LENGTH; i++) begin
            if (i <= l[u]) begin
              elp[u][i] = index_of[elp[u][i]];
            end
          end
          for (i = 0; i <= MAX_LENGTH; i++) begin
            if (i <= l[u]) begin
              regA[i] = elp[u][i]; 
            end
          end
          count = 0;
          for (i = 1; i < POLY_LENGTH; i++) begin
            q = 1;
            for (j = 1; j <= MAX_CAL_POLY_ORDER; j++) begin
              if (j <= l[u]) begin
                if ((regA[j] != -1) && (regA[j] != 65535)) begin
                  regA[j] = (regA[j] + j) % n;
                  q = q ^ alpha_to[regA[j]];
                end
              end
            end
            if (!q) begin
              root[count] = i;
              loc[count] = n - i;
              count = count + 1;
            end
          end
        end
        chienComplete = 1;
      end

      STATE_CORRECT: begin
        if (count == l[u]) begin
          for (i = 0; i < TWO_T; i++) begin
            if (i < l[u]) begin
              recd[loc[i]] = recd[loc[i]] ^ 1;
            end
          end
        end
        correctComplete = 1;
      end

      STATE_DONE: begin
        completeReg$in_ = 1;
      end

      default: begin
      end
    endcase
  end

endmodule  //EccDecodeControl

//-----------------------------------------------------------------------------
// EccDecodeControlBm
// ECC Decoder Berlekamp-Massey Control Unit
//-----------------------------------------------------------------------------
module EccDecodeControlBm
(
  input wire [0:0] clk,         // Clock
  input wire [0:0] reset,       // Reset to a known good state
  input wire [0:0] start,       // Start Berlekamp Massey pasokon
  input wire [15:0] syndrome0,  // syndrome[NumError*2+1]
  input wire [15:0] syndrome1,
  input wire [15:0] syndrome2,
  input wire [15:0] syndrome3,
  input wire [15:0] syndrome4,
  output reg [0:0] complete,    // Pasokon completes
  output reg [0:0] elpValid,    // ELP validity
  output reg [15:0] elp0,       // ErrorLocationPolynomial[NumError+1]
  output reg [15:0] elp1,
  output reg [15:0] elp2
);

  // Galois field arithmetic look-up table
  wire [15:0] alpha_to[0:15];
  wire [15:0] index_of[0:15];

  assign alpha_to[0] = 1;    // 0001 Polynomial 0(x**3) + 0(x**2) + 0(x**1) + 1
  assign alpha_to[1] = 2;    // 0010 Polynomial 0(x**3) + 0(x**2) + 1(x**1) + 0
  assign alpha_to[2] = 4;    // 0100 Polynomial 0(x**3) + 1(x**2) + 0(x**1) + 0
  assign alpha_to[3] = 8;    // 1000 Polynomial 1(x**3) + 0(x**2) + 0(x**1) + 0
  assign alpha_to[4] = 3;    // 0011
  assign alpha_to[5] = 6;    // 0110
  assign alpha_to[6] = 12;   // 1100
  assign alpha_to[7] = 11;   // 1011
  assign alpha_to[8] = 5;    // 0101
  assign alpha_to[9] = 10;   // 1001
  assign alpha_to[10] = 7;   // 0111
  assign alpha_to[11] = 14;  // 1110
  assign alpha_to[12] = 15;  // 1111
  assign alpha_to[13] = 13;  // 1101
  assign alpha_to[14] = 9;   // 1001
  assign alpha_to[15] = 0;

  assign index_of[0] = -1;   // null or zero
  assign index_of[1] = 0;    // alpha**0
  assign index_of[2] = 1;    // alpha**1
  assign index_of[3] = 4;    // alpha**2
  assign index_of[4] = 2;    // alpha**3
  assign index_of[5] = 8;    // alpha**4
  assign index_of[6] = 5;    // alpha**5
  assign index_of[7] = 10;   // alpha**6
  assign index_of[8] = 3;    // alpha**7
  assign index_of[9] = 14;   // alpha**8
  assign index_of[10] = 9;   // alpha**9
  assign index_of[11] = 7;   // alpha**10
  assign index_of[12] = 6;   // alpha**11
  assign index_of[13] = 13;  // alpha**12
  assign index_of[14] = 11;  // alpha**13
  assign index_of[15] = 12;  // alpha**14

  // Local parameters
  localparam NUM_ERROR_CORRECT = 2;
  localparam NUM_ROOT = 2 * NUM_ERROR_CORRECT;
  localparam MAX_LENGTH = 4;
  localparam FINITE_FILED_EXTEND_POWER = 4;
  localparam FINITE_FILED_ORDER = 2 ** 4;
  localparam n = 16'b0000000000001111; // FINITE_FIELD_ORDER - 1

  localparam STATE_IDLE = 0;
  localparam STATE_INIT = 1;
  localparam STATE_NEXT_ITERATE = 2;
  localparam STATE_CAL_POLY_SET = 3;  // Set Connection Algorithmic Polynomial
  localparam STATE_SEARCH_LAST_DISCREP = 4;
  localparam STATE_CAL_POLY_REWIND = 5;
  localparam STATE_CAL_POLY_CALC = 6;
  localparam STATE_UPDATE_LAST_DISCREP = 7;
  localparam STATE_COMPUTE_DISCREP = 8;
  localparam STATE_DONE = 9;

  // Register declarations
  reg [3:0] current_state__0;
  reg [3:0] current_state__1;
  reg [3:0] next_state__0;

  reg [15:0] s[0:4];         // Syndrome
  reg [15:0] elp[0:4][0:2];  // Error Location Polynomial
  reg [4:0] u;               // Iteration step
  reg [15:0] l[0:5];         // Length of polynomial
  reg [15:0] d[0:5];         // Discrepancy
  reg [15:0] u_lu[0:4];      // Last nonzero discrepancy degree differential
  reg [4:0] q;               // Last nonzero discrepancy degree

  integer i;
  integer j;

  reg [0:0] nextIterateSetup;   // Debounce STATE_NEXT_ITERATE
  reg [0:0] calPolyCommence;    // Debounce STATE_CAL_POLY_SET
  reg [0:0] discrepCalculated;  // Debounce STATE_COMPUTE_DISCREP

  // State registers
  wire [0:0] state$reset;
  wire [0:0] state$clk;
  wire [3:0] state$out;
  reg [3:0] state$in_;

  RegRst_0xd3f9810c state
  (
    .clk (state$clk),
    .in_ (state$in_),
    .out (state$out),
    .reset (state$reset)
  );

  // Signal connections
  assign state$clk = clk;
  assign state$reset = reset;

  // Initialize state
  initial begin
    state$in_ = 0;
    complete = 0;
    elpValid = 0;
  end

  // State transitions
  always @ (*) begin
    current_state__0 = state$out;
    next_state__0 = state$out;

    case (current_state__0)
      STATE_IDLE: begin
        if (start) begin
          s[0] = syndrome0;  // Replicate input syndrome into array
          s[1] = syndrome1;
          s[2] = syndrome2;
          s[3] = syndrome3;
          s[4] = syndrome4;
          next_state__0 = STATE_INIT;
        end
      end

      STATE_INIT: begin
        nextIterateSetup = 0;
        next_state__0 = STATE_NEXT_ITERATE;
      end

      STATE_NEXT_ITERATE: begin
        if (nextIterateSetup) begin
          if ((d[u] == -1) || (d[u] == 65535)) begin
            next_state__0 = STATE_CAL_POLY_SET;
          end
          else begin
            next_state__0 = STATE_SEARCH_LAST_DISCREP;
          end
        end
      end

      STATE_CAL_POLY_SET: begin
        if (calPolyCommence) begin
          next_state__0 = STATE_UPDATE_LAST_DISCREP;
          calPolyCommence = 0;
        end
      end

      STATE_SEARCH_LAST_DISCREP: begin
        next_state__0 = STATE_CAL_POLY_REWIND;
      end

      STATE_CAL_POLY_REWIND: begin
        next_state__0 = STATE_CAL_POLY_CALC;
      end

      STATE_CAL_POLY_CALC: begin
        next_state__0 = STATE_UPDATE_LAST_DISCREP;
      end

      STATE_UPDATE_LAST_DISCREP: begin
        next_state__0 = STATE_COMPUTE_DISCREP;
      end

      STATE_COMPUTE_DISCREP: begin
        if ((discrepCalculated)) begin
          if ((u < NUM_ROOT) && (l[u+1] < NUM_ERROR_CORRECT+1)) begin
            nextIterateSetup = 0;
            next_state__0 = STATE_NEXT_ITERATE;
          end
          else begin
            next_state__0 = STATE_DONE;
          end
          discrepCalculated = 0;
        end
      end

      STATE_DONE: begin
        next_state__0 = STATE_DONE;
      end

      default: begin
      end
    endcase

    state$in_ = next_state__0;
  end

  // State outputs
  always @ (*) begin
    current_state__1 = state$out;

    case (current_state__1)
      STATE_IDLE: begin
      end

      STATE_INIT: begin
        d[0] = 0;
        d[1] = s[1];
        for (i = 1; i < NUM_ROOT; i++) begin
          elp[0][i] = -1;
          elp[1][i] = 0;
        end
        elp[0][0] = 0;
        elp[1][0] = 1;  // icarus verilog 12 bug where it would be overwritten if placed before the for loop with i>=1
        l[0] = 0;
        l[1] = 0;
        u_lu[0] = -1;
        u_lu[1] = 0;
        u = 0;
      end

      STATE_NEXT_ITERATE: begin
        u = u + 1;
        nextIterateSetup = 1;
      end

      STATE_CAL_POLY_SET: begin
        l[u+1] = l[u];
        for (i = 0; i <= MAX_LENGTH; i++) begin
          if (i <= l[u]) begin
            //elp[u+1][i] = elp[u][i];
            elp[u][i] = index_of[elp[u][i]];
          end
        end
        calPolyCommence = 1;
      end

      STATE_SEARCH_LAST_DISCREP: begin
        q = u - 1;
        for (i = MAX_LENGTH; i > 0; i--) begin
          if (((d[q] == -1) || (d[q] == 65535)) && (q > 0)) begin
            q = q - 1;
          end
        end
        if (q > 0) begin
          j = q;
          for (i = MAX_LENGTH; i > 0; i--) begin
            if (j > 0) begin
              j = j - 1;
              if (((d[j] != -1) && (d[j] != 65535)) && ((u_lu[q] < u_lu[j]) && (u_lu[j] != 65535))) begin
                q = j;
              end
            end
          end
        end
      end

      STATE_CAL_POLY_REWIND: begin
        if (l[u] > l[q] + u - q ) begin
          l[u+1] = l[u];
        end
        else begin
          l[u+1] = l[q] + u + q;
        end
      end

      STATE_CAL_POLY_CALC: begin
        for (i = 0; i < MAX_LENGTH; i++) begin
          elp[u+1][i] = 0;
        end
        for (i = 0; i <= MAX_LENGTH; i++) begin
          if (i <= l[q]) begin
            if ((elp[q][i] != -1) && (elp[q][i] != 65535)) begin
              elp[u+1][i+u-q] = alpha_to[(d[u] + n - d[q] + elp[q][i]) % n];
            end
          end
        end
        for (i = 0; i < MAX_LENGTH; i++) begin
          if (i <= l[u]) begin
            elp[u+1][i] = elp[u+1][i] ^ elp[u][i];
            elp[u][i] = index_of[elp[u][i]];
          end
        end
      end

      STATE_UPDATE_LAST_DISCREP: begin
        u_lu[u+1] = u - l[u+1];
      end

      STATE_COMPUTE_DISCREP: begin
        if (u < NUM_ROOT) begin
          if ((s[u+1] != -1) && (s[u+1] != 65535)) begin
            d[u+1] = alpha_to[s[u+1]];
          end
          else begin
            d[u+1] = 0;
          end
          for (i = 1; i <= MAX_LENGTH; i++) begin
            if (i <= l[u + 1]) begin
              if (((s[u + 1 - i] != -1) && (s[u + 1 - i] != 65535)) && (elp[u + 1][i] != 0)) begin
                d[u + 1] = d[u + 1] ^ alpha_to[(s[u + 1 - i] + index_of[elp[u + 1][i]]) % n];
              end
            end
          end
          d[u + 1] = index_of[d[u + 1]];
        end
        discrepCalculated = 1;
      end

      STATE_DONE: begin
        u = u + 1;
        elp0 = elp[u][0];
        elp1 = elp[u][1];
        elp2 = elp[u][2];
        complete = 1;
        if (l[u] < NUM_ERROR_CORRECT+1) begin
          elpValid = 1;
        end
      end

      default: begin
      end       
    endcase
  end

endmodule  //EccDecodeControlBm

//-----------------------------------------------------------------------------
// RegRst_0xd3f98109
// Register 1 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f98109
(
  input wire [0:0] clk,
  input wire [0:0] in_,
  output reg [0:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 1'b0;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f98109

//-----------------------------------------------------------------------------
// RegRst_0xd3f9810a
// Register 2 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f9810a
(
  input wire [0:0] clk,
  input wire [1:0] in_,
  output reg [1:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 0;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f9810a

//-----------------------------------------------------------------------------
// RegRst_0xd3f9810b
// Register 3 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f9810b
(
  input wire [0:0] clk,
  input wire [2:0] in_,
  output reg [2:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 0;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f9810b

//-----------------------------------------------------------------------------
// RegRst_0xd3f9810c
// Register 4 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f9810c
(
  input wire [0:0] clk,
  input wire [3:0] in_,
  output reg [3:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 0;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f9810c

//-----------------------------------------------------------------------------
// RegRst_0xd3f9810f
// Register 15 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f9810f
(
  input wire [0:0] clk,
  input wire [14:0] in_,
  output reg [14:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 15'b000000000000000;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f9810f

//-----------------------------------------------------------------------------
// RegRst_0xd3f98110
// Register 16 bit width with Reset
//-----------------------------------------------------------------------------
module RegRst_0xd3f98110
(
  input wire [0:0] clk,
  input wire [15:0] in_,
  output reg [15:0] out,
  input wire [0:0] reset
);

  localparam reset_value = 16'b0000000000000000;

  always @ (posedge clk) begin
    if (reset) begin
      out <= reset_value;
    end
    else begin
      out <= in_;
    end
  end
endmodule // RegRst_0xd3f98110

//-----------------------------------------------------------------------------
// RegEn_0x68db79c4ec1d6e50
// Register 1 Bit Width with Enable
//-----------------------------------------------------------------------------
module RegEn_0x68db79c4ec1d6e50
(
  input wire [0:0] clk,
  //input wire [0:0] reset,
  input wire [0:0] in_,
  input wire [0:0] en,
  output reg [0:0] out
);

  always @ (posedge clk) begin
    if (en) begin
      out <= in_;
    end
    else begin
      out <= 0;
    end
  end
endmodule // RegEn_0x68db79c4ec1d6e50

//-----------------------------------------------------------------------------
// RegEn_0x68db79c4ec1d6e5b
// Register 16 Bit Width with Enable
//-----------------------------------------------------------------------------
module RegEn_0x68db79c4ec1d6e5b
(
  input wire [0:0] clk,
  input wire [0:0] reset,
  input wire [15:0] in_,
  input wire [0:0] en,
  output reg [15:0] out
);

  always @ (posedge clk) begin
    if (en) begin
      out <= in_;
    end
    else begin
    end
  end
endmodule // RegEn_0x68db79c4ec1d6e5b

//-----------------------------------------------------------------------------
// Mux_0xa543df60
// Mux for 1 Bit Data with 1 Select Line
//-----------------------------------------------------------------------------
module Mux_0xa543df60
(
  input wire [0:0] clk,
  input wire [0:0] in_$000,
  input wire [0:0] in_$001,
  input wire [0:0] sel,
  output reg [0:0] out
);

  wire [0:0] in_[0:1];
  assign in_[0] = in_$000;
  assign in_[1] = in_$001;

  always @ (*) begin
    out = in_[sel];
  end
endmodule // Mux_0xa543df60

//-----------------------------------------------------------------------------
// Mux_0xa543df63
// Mux for 15 Bit Data with 1 Select Line
//-----------------------------------------------------------------------------
module Mux_0xa543df63
(
  input wire [0:0] clk,
  input wire [14:0] in_$000,
  input wire [14:0] in_$001,
  input wire [0:0] sel,
  output reg [14:0] out
);

  wire [14:0] in_[0:1];
  assign in_[0] = in_$000;
  assign in_[1] = in_$001;

  always @ (*) begin
    out = in_[sel];
  end
endmodule // Mux_0xa543df63

//-----------------------------------------------------------------------------
// Mux_0xa543df64
// Mux for 16 Bit Data with 1 Select Line
//-----------------------------------------------------------------------------
module Mux_0xa543df64
(
  input wire [0:0] clk,
  input wire [0:0] reset,
  input wire [15:0] in_$000,
  input wire [15:0] in_$001,
  input wire [0:0] sel,
  output reg [15:0] out
);

  wire [15:0] in_[0:1];
  assign in_[0] = in_$000;
  assign in_[1] = in_$001;

  always @ (*) begin
    out = in_[sel];
  end
endmodule // Mux_0xa543df64

//-----------------------------------------------------------------------------
// Driver_0x462543a0
// Driver for 1 Bit Data with 1 Select Line
//-----------------------------------------------------------------------------
module Driver_0x462543a0
(
  input wire [0:0] clk,
  input wire [0:0] in,
  input wire [0:0] sel,
  output reg [0:0] out_$000,
  output reg [0:0] out_$001
);

  always @ (*) begin
    if (sel == 0) begin
      out_$000 <= in;
      out_$001 <= 0;
    end
    else begin
      out_$000 <= 0;
      out_$001 <= in;
    end
  end
endmodule // Driver_0x462543a0

`default_nettype wire
