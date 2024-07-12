module alu #(
    parameter INT_W  = 6,
    parameter FRAC_W = 10,
    parameter INST_W = 4,
    parameter DATA_W = INT_W + FRAC_W
)(
    input                     i_clk,
    input                     i_rst_n,
    input signed [DATA_W-1:0] i_data_a,
    input signed [DATA_W-1:0] i_data_b,
    input        [INST_W-1:0] i_inst,
    output                    o_valid,
    output                    o_busy,
    output       [DATA_W-1:0] o_data
);

/****************************   parameters ***********************/

localparam STATE_IDLE    = 2'b00 ;
localparam STATE_COMPUTE = 2'b01 ;
localparam STATE_OUTPUT  = 2'b10 ;

localparam INSTRUCT_ADD_FX = 4'b0000 ;
localparam INSTRUCT_SUB_FX = 4'b0001 ;
localparam INSTRUCT_MUL    = 4'b0010 ;
localparam INSTRUCT_MAC    = 4'b0011 ;
localparam INSTRUCT_GELU   = 4'b0100 ;
localparam INSTRUCT_CLZ    = 4'b0101 ;
localparam INSTRUCT_LRCW   = 4'b0110 ;
localparam INSTRUCT_LFSR   = 4'b0111 ;
localparam INSTRUCT_ADD_FP = 4'b1000 ;
localparam INSTRUCT_SUB_FP = 4'b1001 ;


/*************************** wire and reg ******************************/

//global
reg signed [DATA_W-1:0] o_data_w, o_data_r ;
reg [5:0] constant  ;

   
integer i, j ;

//state
reg o_busy_w, o_valid_w, o_busy_r, o_valid_r, o_data_signed ;

//FSM
reg [3:0] current_state ;
reg [3:0] next_state ;

//I0  ADD_FX
reg  signed [DATA_W-1:0] ADD_FX_ans ;
wire signed [DATA_W-1:0] ADD_FX_ans_pre ;


//I1  SUB_FX
reg  signed [DATA_W-1:0] SUB_FX_ans ;
wire signed [DATA_W-1:0] SUB_FX_ans_pre ;

//I2 MUL
reg  signed [DATA_W-1:0] MUL_ans ;
wire signed [2*DATA_W-1:0] MUL_ans_pre ;


//I3 MAC
reg  signed [DATA_W-1:0] MAC_ans ;
wire signed [2*DATA_W-1:0] MAC_ans_pre ;

//I4 GELU
reg signed [DATA_W-1:0] GELU_ans ;
wire signed [3*DATA_W-1:0] GELU_cal_1 ;
wire signed [5*DATA_W-1:0] GELU_cal_2 ;
reg signed [DATA_W-1:0] GELU_round1, GELU_round2, GELU_round2_pre ;
wire signed [DATA_W-1:0] GELU_cal_3 ;
wire signed [3*DATA_W-1:0] GELU_cal_4 ;

//I5 CLZ
reg signed [DATA_W-1:0] CLZ_ans ;
wire signed [3:0] CLZ_ans_cal ;
wire vld_I5 ;

//I6 LRCW
reg [DATA_W-1:0] LRCW_ans ;
wire [3:0] c_pop ;
wire [DATA_W-1:0] LRCW_shift [0:15] ; //[number] [bit]
wire [DATA_W-1:0] LRCW_complement [0:15] ;
wire [DATA_W-1:0] LRCW_f [0:15] ;
wire [DATA_W-1:0] LRCW_b [0:15] ;
wire [DATA_W-1:0] LRCW_ans_pre [0:15] ;

//I7 LFSR
reg signed [DATA_W-1:0] LFSR_ans ;
wire LFSR_pre_15 [0:6] ;
wire LFSR_pre_13 [0:6] ;
wire LFSR_pre_12 [0:6] ;
wire LFSR_pre_10 [0:6] ;
wire LFSR_lsb [0:7] ;
wire [DATA_W-2:0] LFSR_shift [0:8] ;
wire [DATA_W-1:0] LFSR_pre [0:8] ;

//I8 & I9 : ADD_FP, SUB_FP
reg signed [DATA_W-1:0] FP_ans, FP_ans_pre ;

wire [23:0] fract_a, fract_b ;
reg  [23:0] fract_a_shift, fract_b_shift, fract_ans, fract_ans_1, fract_ans_2 ;

wire sign_a, sign_b, norm_vld ;
reg  sign_ans ;

wire [4:0] exponent_a, exponent_b ;
reg  [4:0] exponent_ans, exponent_ans_1, exponent_ans_2 ;

wire [3:0] pos ;
wire [15:0] pre_frac ;

wire guard_bit, round_bit, sticky_bit ;

//************************************FSM*************************************//
//reg [3:0] current_state ;
always @( posedge i_clk , negedge i_rst_n ) begin
    if ( !i_rst_n ) current_state <= STATE_IDLE ;
    else current_state <= next_state ;
end

//reg [3:0] next_state ;
always @(*) begin
    case( current_state )
        STATE_IDLE: begin
            o_busy_w = 1'b1 ;
            o_valid_w = 1'b0 ;
            next_state = STATE_COMPUTE;
        end
        STATE_COMPUTE: begin
            o_busy_w = 1'b1 ;
            o_valid_w = 1'b0 ;
            next_state = STATE_OUTPUT ;
        end
        STATE_OUTPUT: begin;
            o_busy_w = 1'b0 ;
            o_valid_w = 1'b1 ;
            next_state = STATE_IDLE ;
        end
        default: begin
            next_state = STATE_IDLE ;
        end
    endcase
end

//**********************OUTPUT********************//

assign o_data = o_data_r ;
assign o_valid = o_valid_r ;
assign o_busy = o_busy_r ;


always @(posedge i_clk, negedge i_rst_n ) begin
    if ( !i_rst_n ) begin
        o_data_r <= 1'b0 ;
    end
    else begin
        o_data_r <= o_data_w ; 
    end
end

always @(posedge i_clk, negedge i_rst_n ) begin
    if ( !i_rst_n ) begin
        o_valid_r <= 1'b0 ;
    end
    else begin
        o_valid_r <= o_valid_w ; 
    end
end

always @(posedge i_clk, negedge i_rst_n ) begin
    if ( !i_rst_n ) begin
        o_busy_r <= 1'b0 ;
    end
    else begin
        o_busy_r <= o_busy_w ; 
    end
end

always @(*) begin
    case (i_inst)
        INSTRUCT_ADD_FX : begin
            o_data_w = ADD_FX_ans ;
        end 
        INSTRUCT_SUB_FX : begin
            o_data_w = SUB_FX_ans ;
        end
        INSTRUCT_MUL : begin
            o_data_w = MUL_ans ; 
        end
        INSTRUCT_MAC : begin
            o_data_w = MAC_ans ;
        end
        INSTRUCT_GELU : begin
            o_data_w = GELU_ans ;
        end
        INSTRUCT_CLZ : begin
            o_data_w = CLZ_ans ;
        end
        INSTRUCT_LRCW : begin
            o_data_w = LRCW_ans ;
        end
        INSTRUCT_LFSR : begin
            o_data_w = LFSR_ans ;
        end
        INSTRUCT_ADD_FP : begin
            o_data_w = FP_ans ;
        end
        INSTRUCT_SUB_FP : begin
            o_data_w = FP_ans ;
        end
        default: o_data_w = 16'h0000 ;
    endcase
end

/**************************************************************************************/
//I0 INSTRUCT_ADD_FX
assign ADD_FX_ans_pre = i_data_a + i_data_b ;

always @(*) begin
    if ( {i_data_a[15], i_data_b[15], ADD_FX_ans_pre[15]} == 3'b110 ) 
        ADD_FX_ans = 16'b0111111111111111 ;
    else if ( {i_data_a[15], i_data_b[15], ADD_FX_ans_pre[15]} == 3'b001 ) 
        ADD_FX_ans = 16'b1000000000000000 ;
    else 
        ADD_FX_ans = ADD_FX_ans_pre ;
end

/**************************************************************************************/
//I1 INSTRUCT_SUB_FX
assign SUB_FX_ans_pre = i_data_a - i_data_b ;

always @(*) begin
    if ( {i_data_a[15], i_data_b[15], SUB_FX_ans_pre[15]} == 3'b100 )
        SUB_FX_ans = 16'b0111111111111111 ;
    else if ( {i_data_a[15], i_data_b[15], SUB_FX_ans_pre[15]} == 3'b011 )
        SUB_FX_ans = 16'b1000000000000000 ;
    else 
        SUB_FX_ans = SUB_FX_ans_pre ;   
end

/**************************************************************************************/
//I2 INSTRUCT_MUL
assign MUL_ans_pre = i_data_a * i_data_b ;

always @(*) begin
    if ( (MUL_ans_pre[31] == 0) && (MUL_ans_pre > 32'b000000_011111_1111111111_0000000000) )
        MUL_ans = 16'b011111_1111111111 ;
    else if ( (MUL_ans_pre[31] == 1) && (MUL_ans_pre < 32'b111111_100000_0000000000_0000000000) )
        MUL_ans = 16'b100000_0000000000 ;
    else MUL_ans = MUL_ans_pre ;
end

/**************************************************************************************/
//I3 INSTRUCT_MAC
assign MAC_ans_pre = $signed ( {6*o_data_r[15], o_data_r, 10'd0} ) + i_data_a * i_data_b ;

always @(*) begin
    if ( (MAC_ans_pre[31] == 0) && MAC_ans_pre > 32'b000000_011111_1111111111_0000000000 )
        MAC_ans = 16'b0111111111111111 ;
    else if ( (MAC_ans_pre[31] == 1) && (MAC_ans_pre < 32'b111111_100000_0000000000_0000000000) )
        MAC_ans = 16'b100000_0000000000 ;
    else MAC_ans = MAC_ans_pre [25:10] ;
end

/**************************************************************************************/
//I4 INSTRUCT_GELU
assign GELU_cal_1 = $signed(48'b000000_000000_000001_0000000000_0000000000_0000000000) + ($signed(16'b000000_0000101110)*i_data_a*i_data_a) ;//48
assign GELU_cal_2 = $signed(16'b000000_1100110001) * i_data_a * GELU_cal_1 ;//80

//GELU_round1   before tanh
// always @(*) begin
//     if ( (GELU_cal_2[79] == 0) && GELU_cal_2 > $signed(80'b000000_000000_000000_000000_011111_1111111111_0000000000_0000000000_0000000000_0000000000)) 
//         GELU_round1 = 16'b011111_1111111111 ;
//     else if ( (GELU_cal_2[79] == 1) && GELU_cal_2 > $signed (80'b111111_111111_111111_111111_100000_0000000000_0000000000_0000000000_0000000000_0000000000) )
//         GELU_round1 = 16'b100000_0000000000 ;
//     else GELU_round1 = GELU_cal_2[55:40] + GELU_cal_2[39] ;
// end
always @(*) begin
    GELU_round1 = GELU_cal_2[55:40] + GELU_cal_2[39] ;
end



//GELU_round2   after tanh
always @(*) begin
    if ( GELU_round1 < $signed(16'b111110_1000000000) ) // x <-1.5
        GELU_round2 = 16'b111111_0000000000 ;

    else if ( ( GELU_round1 >= $signed(16'b111110_1000000000) ) && ( GELU_round1 < $signed(16'b111111_1000000000) ) ) begin // -0.5 > x >= -1.5
        GELU_round2_pre = ( $signed(GELU_round1) >>> 1 ) ;
        GELU_round2 = GELU_round2_pre + GELU_round1[0] - $signed(16'b000000_0100000000) ; 
        // GELU_round2 = ( $signed(GELU_round1) >>> 1 ) + GELU_round1[0] - $signed(16'b000000_0100000000) ; 
    end

    else if ( ( GELU_round1 >= $signed(16'b111111_1000000000) ) && ( GELU_round1 < $signed(16'b000000_1000000000) ) ) // 0.5 > x >= 0.5
        GELU_round2 = GELU_round1 ;

    else if ( ( GELU_round1 >= $signed(16'b000000_1000000000) ) && ( GELU_round1 < $signed(16'b000001_1000000000) ) ) begin // 1.5 > x >= 0.5
        GELU_round2_pre = ( $signed(GELU_round1) >>> 1 ) ;
        GELU_round2 = GELU_round2_pre + GELU_round1[0] + $signed(16'b000000_0100000000) ;
        // GELU_round2 = ( $signed(GELU_round1) >>> 1 ) + GELU_round1[0] + $signed(16'b000000_0100000000) ;

    end

    else if ( GELU_round1 >= $signed(16'b000001_1000000000) )
        GELU_round2 = 16'b000001_0000000000 ;
end

assign GELU_cal_3 =  16'b000001_0000000000 + GELU_round2  ;
assign GELU_cal_4 = $signed(16'b000000_1000000000) * i_data_a * GELU_cal_3 ;//48

//GELU_round3 final
// always @(*) begin
//     if      ( ( GELU_cal_3[47] == 0 ) && (GELU_cal_3 > 48'b000000_000000_011111_1111111111_0000000000_0000000000) )
//         GELU_ans = 16'b011111_1111111111 ;
//     else if ( ( GELU_cal_3[47] == 1 ) && (GELU_cal_3 < 48'b111111_111111_100000_0000000000_0000000000_0000000000) )
//         GELU_ans = 16'b100000_0000000000 ;
//     else GELU_ans = GELU_cal_3[35:20] + GELU_cal_3[19] ; 
// end
always @(*) begin
    GELU_ans = GELU_cal_4[35:20] + GELU_cal_4[19] ;
end


/**************************************************************************************/
//I5 INSTRUCT_CLZ
PEC16 use_I5 ( .D_2(i_data_a), .Q_2(CLZ_ans_cal), .VLD_2(vld_I5) ) ;

always @(*) begin
    CLZ_ans = 16'd15 - CLZ_ans_cal ;
end



/*************************************************************************************/
//I6 INSTRUCT_LRCW
assign c_pop = i_data_a[0] + i_data_a[1] + i_data_a[2] + i_data_a[3] + i_data_a[4] + i_data_a[5] + i_data_a[6] + i_data_a[7] + i_data_a[8] + i_data_a[9]
             + i_data_a[10] + i_data_a[11] + i_data_a[12] + i_data_a[13] + i_data_a[14] + i_data_a[15] ;

genvar c_pop_n ;
generate
    for ( c_pop_n = 0 ; c_pop_n < 16 ; c_pop_n = c_pop_n + 1 ) begin
        assign LRCW_shift[c_pop_n] = i_data_b <<< c_pop_n ;
        // assign LRCW_complement_n[c_pop_n] = 4'd15 - c_pop_n ;
        assign LRCW_complement[c_pop_n] = { {(c_pop_n){1'b0}}, ~i_data_b[15:15-c_pop_n]} ;
        assign LRCW_ans_pre[c_pop_n] = LRCW_shift[c_pop_n] + LRCW_complement[c_pop_n-1] ;  
        // assign LRCW_f[c_pop_n] = LRCW_shift[c_pop_n][15:c_pop_n] ;
        // assign LRCW_b[c_pop_n] = LRCW_complement[c_pop_n][c_pop_n-1:0] ;
        // assign LRCW_ans_pre[c_pop_n] = { LRCW_f[c_pop_n], LRCW_b[c_pop_n] } ;
        // assign LRCW_ans_pre[c_pop_n] = { LRCW_shift[c_pop_n][15:c_pop_n], LRCW_complement[c_pop_n][c_pop_n-1:0] } ;
    end
endgenerate


always @(*) begin
    case ( c_pop ) 
        0  : begin LRCW_ans = i_data_b ; end 
        1  : begin LRCW_ans = LRCW_ans_pre[ 1] ; end 
        2  : begin LRCW_ans = LRCW_ans_pre[ 2] ; end 
        3  : begin LRCW_ans = LRCW_ans_pre[ 3] ; end 
        4  : begin LRCW_ans = LRCW_ans_pre[ 4] ; end 
        5  : begin LRCW_ans = LRCW_ans_pre[ 5] ; end 
        6  : begin LRCW_ans = LRCW_ans_pre[ 6] ; end 
        7  : begin LRCW_ans = LRCW_ans_pre[ 7] ; end 
        8  : begin LRCW_ans = LRCW_ans_pre[ 8] ; end 
        9  : begin LRCW_ans = LRCW_ans_pre[ 9] ; end 
        10 : begin LRCW_ans = LRCW_ans_pre[10] ; end 
        11 : begin LRCW_ans = LRCW_ans_pre[11] ; end 
        12 : begin LRCW_ans = LRCW_ans_pre[12] ; end 
        13 : begin LRCW_ans = LRCW_ans_pre[13] ; end 
        14 : begin LRCW_ans = LRCW_ans_pre[14] ; end 
        15 : begin LRCW_ans = LRCW_ans_pre[15] ; end 
        default: begin LRCW_ans = i_data_b ; end
    endcase
end



/*************************************************************************************/
//I7 INSTRUCT_LFSR

assign LFSR_lsb[0] = i_data_a[15] ^ i_data_a[13] ^ i_data_a[12] ^ i_data_a [10] ;
genvar LFSR_lsb_var ;
generate
    for ( LFSR_lsb_var=0 ; LFSR_lsb_var<=7 ; LFSR_lsb_var=LFSR_lsb_var+1 ) begin
        assign LFSR_pre_15[LFSR_lsb_var] = LFSR_pre[LFSR_lsb_var+1][15] ;
        assign LFSR_pre_13[LFSR_lsb_var] = LFSR_pre[LFSR_lsb_var+1][13] ;
        assign LFSR_pre_12[LFSR_lsb_var] = LFSR_pre[LFSR_lsb_var+1][12] ;
        assign LFSR_pre_10[LFSR_lsb_var] = LFSR_pre[LFSR_lsb_var+1][10] ;
        assign LFSR_lsb[LFSR_lsb_var + 1] = ( ( (LFSR_pre_15[LFSR_lsb_var] ^ LFSR_pre_13[LFSR_lsb_var]) ^ LFSR_pre_12[LFSR_lsb_var] ) ^ LFSR_pre_10[LFSR_lsb_var] ) ; 
    end
endgenerate



genvar LFSR_shift_var ;
generate
    for ( LFSR_shift_var=0 ; LFSR_shift_var<9 ; LFSR_shift_var=LFSR_shift_var+1) begin
        assign LFSR_shift[LFSR_shift_var] = LFSR_pre[LFSR_shift_var][DATA_W-2:0] ;
    end
endgenerate

assign LFSR_pre[0] = i_data_a ;
genvar iteration ; 
generate
    for( iteration=0 ; iteration<8 ; iteration=iteration+1 ) begin
        assign LFSR_pre[iteration+1] = { LFSR_shift[iteration][DATA_W-2:0], LFSR_lsb[iteration] } ;
    end
endgenerate

always @(*) begin
    LFSR_ans = LFSR_pre[i_data_b] ;
end



/*************************************************************************************/
//I8 INSTRUCT_ADD_FP 、 I9 INSTRUCT_SUB_FP

//laoding
assign fract_a = { 1'b0, 1'b1, i_data_a[9:0], 12'd0 } ; //24
assign fract_b = { 1'b0, 1'b1, i_data_b[9:0], 12'd0 } ;

assign exponent_a = { i_data_a[14:10] } ;
assign exponent_b = { i_data_b[14:10] } ;

assign sign_a = i_data_a[15] ;
assign sign_b = ( i_inst == INSTRUCT_ADD_FP ) ? i_data_b[15] : ~i_data_b[15] ;

//align
always @(*) begin
    if ( exponent_a > exponent_b ) begin
        fract_b_shift = fract_b >> ( exponent_a - exponent_b ) ;
        fract_a_shift = fract_a ;
        exponent_ans = exponent_a ;
    end
    else if ( exponent_a < exponent_b ) begin
        fract_a_shift = fract_a >> ( exponent_b - exponent_a ) ;
        fract_b_shift = fract_b ;
        exponent_ans = exponent_b ;
    end
    else begin
        fract_a_shift = fract_a ;
        fract_b_shift = fract_b ;
        exponent_ans = exponent_a ;
    end
end

//add significiant
always @(*) begin
    if ( sign_a == sign_b ) begin
        fract_ans = fract_a_shift + fract_b_shift ;
        sign_ans = sign_a  ;
    end
    else begin
        if ( fract_a_shift >= fract_b_shift ) begin
            fract_ans = fract_a_shift - fract_b_shift ;
            sign_ans = sign_a ;
        end
        else begin
            fract_ans = fract_b_shift - fract_a_shift ;
            sign_ans = sign_b ;
        end
    end
end

//overflow
always @(*) begin
    if ( fract_ans[23] == 1 ) begin
        fract_ans_1 = fract_ans >> 1 ;
        exponent_ans_1 = exponent_ans + 1 ; 
    end
    else begin
        fract_ans_1 = fract_ans ;
        exponent_ans_1 = exponent_ans ;
    end
end

//normalization
assign pre_frac = {6'd0, fract_ans_1[21:12]} ;
PEC16 use_I8 ( .D_2(pre_frac), .Q_2(pos), .VLD_2(norm_vld) ) ;

always @(*) begin
    if ( fract_ans_1 == 24'b0 ) begin
        fract_ans_2 = 24'b0 ;
        exponent_ans_2 = 5'b0 ;
    end
    else begin
        if ( fract_ans_1[22] == 0 ) begin
            fract_ans_2 = fract_ans_1 << ( 5'd10 - pos ) ;//fraction 10 bit - first 1 index
            exponent_ans_2 = exponent_ans_1 + pos - 5'd10 ;
            FP_ans_pre = { sign_ans, exponent_ans_2, fract_ans_2[21:12] } ;
        end
        else if ( fract_ans_1[22] ) begin
            fract_ans_2 = fract_ans_1 ;
            exponent_ans_2 = exponent_ans_1 ;
            FP_ans_pre = { sign_ans, exponent_ans_2, fract_ans_2[21:12] } ; 
        end                    
    end
end

//round
assign guard_bit = fract_ans_2[12] ;
assign round_bit = fract_ans_2[11] ;
assign sticky_bit = ( fract_ans_2[10:0] > 0 ) ? 1'b1 : 1'b0 ;

always @(*) begin 
    if ( round_bit == 1 && ( guard_bit == 1 || sticky_bit == 1 ) )
        FP_ans = FP_ans_pre + 1 ;
    else
        FP_ans = FP_ans_pre ;
end


endmodule





module PEC16 (
    input [15:0] D_2, 
    output [3:0] Q_2, 
    output VLD_2 
);
wire VLD_2_H, VLD_2_L ;
wire [2:0] Q_2_H, Q_2_L ;

PEC8 p0 ( .D_1(D_2[15:8]), .Q_1(Q_2_H), .VLD_1(VLD_2_H) ) ;
PEC8 p1 ( .D_1(D_2[7:0]), .Q_1(Q_2_L), .VLD_1(VLD_2_L) ) ;

assign Q_2[3] = VLD_2_H ;
assign Q_2[2:0] = VLD_2_H ? Q_2_H : Q_2_L ;
assign VLD_2 = VLD_2_H | VLD_2_L ;

endmodule

module PEC8 (
    input [7:0] D_1, 
    output [2:0] Q_1, 
    output VLD_1 
);
wire VLD_1_H, VLD_1_L ;
wire [1:0] Q_1_H, Q_1_L ;

PEC4 p0 ( .D_0(D_1[7:4]), .Q_0(Q_1_H), .VLD_0(VLD_1_H) ) ;
PEC4 p1 ( .D_0(D_1[3:0]), .Q_0(Q_1_L), .VLD_0(VLD_1_L) ) ;

assign Q_1[2] = VLD_1_H ;
assign Q_1[1:0] = VLD_1_H ? Q_1_H : Q_1_L ;
assign VLD_1 = VLD_1_H | VLD_1_L ;

endmodule

module PEC4 (
	input [3:0] D_0,
	output [1:0] Q_0,
	output VLD_0 
);

assign Q_0[0] = ( ~D_0[2] & D_0[1] ) | D_0[3] ;
assign Q_0[1] = D_0[2] | D_0[3] ;
assign VLD_0 = D_0[0] | D_0[1] | D_0[2] | D_0[3] ;

endmodule