// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "i2c_master_defines.v"

module myi2c_master_byte_ctrl (
  clk,nReset,ena,rst
  clk_cnt,start,stop,read,write,
  ack_in,din,cmd_ack,ack_out,dout,i2c_busy,i2c_al,
  scl_o,scl,oen,scl_i,sda_i,sda_o,sda_oen
);
  
input clk;
input nReset;
input rst;
input ena;

input [15:0] wire clk_cnt;

//conmand and data input
input start;
input stop;
input read;
input write;
input ack_in;
input [7:0] din;

// status outputs
output  reg   cmd_ack;
output  reg   ack_out;
output  i2c_busy;
output  i2c_al;
output [7:0] dout;

// I2C signals and pad control
input  scl_i;
output scl_o;
output scl_oen;
input  sda_i;
output sda_o;
output sda_oen;


// statemachine
parameter [4:0] ST_IDLE  = 5'b0_0000;
parameter [4:0] ST_START = 5'b0_0001;
parameter [4:0] ST_READ  = 5'b0_0010;
parameter [4:0] ST_WRITE = 5'b0_0100;
parameter [4:0] ST_ACK   = 5'b0_1000;
parameter [4:0] ST_STOP  = 5'b1_0000;
// signals for state machine
wire       go;
reg  [2:0] bit_cnt;  //剩余待发送比特数据的个数
wire       cnt_done;  //发送完成计数器


// signals for bit_controller
reg  [3:0] core_cmd;  //传送到下一层Bitctrl的控制命令
reg        core_txd;  //传送到bit控制器的当前发送的bit数据
wire       core_ack； //
wire       core_rxd;  //

// signals for bit_controller
reg  [3:0] core_cmd;  //传送个bit控制器的控制命令
reg        core_txd;  //并串转化后传到下一层的bit的发送数据
wire       core_ack, core_rxd;
// signals for shift register
reg [7:0] sr; //8bit shift register
reg       shift, ld;


i2c_master_bit_ctrl bit_controller (
	.clk     ( clk      ),//输入时钟
	.rst     ( rst      ),//异步复位信号
	.nReset  ( nReset   ),//同步复位信号
	.ena     ( ena      ),//模块使能信号
	.clk_cnt ( clk_cnt  ),//分频计数器
	.cmd     ( core_cmd ),//传送到下一层Bitctrl的控制命令
	.cmd_ack ( core_ack ),//下层控制器的指令应答信号
	.busy    ( i2c_busy ),//busy信号
	.al      ( i2c_al   ),//仲裁失败信号
	.din     ( core_txd ),//输入发送的bit数据
	.dout    ( core_rxd ),//接收bit信号
	.scl_i   ( scl_i    ),
	.scl_o   ( scl_o    ),
	.scl_oen ( scl_oen  ),
	.sda_i   ( sda_i    ),
	.sda_o   ( sda_o    ),
	.sda_oen ( sda_oen  )
);

/*状态机工作信号 */
assign go = (read | write | stop ) & ~cmd_ack;

// assign dout output to shift-register
assign dout = sr;

// generate shift register
always @(posedge clk or negedge nReset)
begin
  if (!nReset)
    sr <= #1 8'h0;
  else if (ld)
    sr <= #1 din;
  else if (shift)
    sr <= #1 {sr[6:0], core_rxd};
end

// generate counter
always @(posedge clk or negedge nReset)
begin
  if (!nReset)
    bit_cnt <= #1 3'h0;
  else if (ld)
    bit_cnt <= #1 3'h7;
  else if (shift)
    bit_cnt <= #1 bit_cnt - 3'h1;
end


assign cnt_done = ~(|bit_cnt);
//
// state machine
//
reg [4:0] c_state; // synopsys enum_state
















endmodule