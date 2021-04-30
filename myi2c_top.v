// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

`include "i2c_master_defines.v"

module myi2c_top(PCLK,PCLKG,PREST_n,PADDR,PSEL,PWRITE,PENABLE,rst
                  PWDATA,PRDATA,PREADY,PSLVEER,inta_o,
                  scl_pad_i,scl_pad_o,scl_padoen_o,sda_pad_i,sda_pad_o,sda_padoen_o);

/*CLK��& Rest Signal*/
input wire PCLK;
input wire PCLKG;
input wire PREST_n;
input wire rst;

input wire PSEL;
/*31��12 ��APB�϶���һ���ģ���0x40000004 �����������һ������0x4000�ǲ���ģ�������������һ��0*/
input wire [11:2] PADDR;/*�����źų���4��APB����ģ��ʶ��ʹ��WORD��ַ��һ��REG��32bit*/
input wire PWRITE;
input wire PENABLE;
input wire [31:0] PWDATA;

output wire [31:0] PRDATA;
output wire PREADY;
output wire PSLVEER;
output reg  inta_o;//Interrupt signal

/*I2C  IO PAD */
input  wire scl_pad_i;       // SCL-line input
output wire scl_pad_o;       // SCL-line output (always 1'b0)
output wire scl_padoen_o;    // SCL-line output enable (active low)
	// i2c data line
input  sda_pad_i;       // SDA-line input
output sda_pad_o;       // SDA-line output (always 1'b0)
output sda_padoen_o;    // SDA-line output enable (active low)

assign PREADY = 1'b1;
assign PSLVEER = 1'b0;

/*En �򿪵�ʱ�����͵��㡣enΪ�͵�ʱ���Ǹ��裬���ⲿ������VCC*/
assign scl_pad_o = 1'b0;      
assign sda_padoen_o = 1'b0;

	// registers ���Ͷ����Ϊ�﷨����
reg  [15:0] prer; // clock prescale register                0x00
reg  [ 7:0] ctr;  // control register ����ģ��ʹ�ܺ��ж�ʹ��  0x04
reg  [ 7:0] txr;  // transmit register ���ͻ��� RO           0x08
wire [ 7:0] rxr;  // receive register ���ջ���               0x0C
reg  [ 7:0] cr;   // command register �ײ�Э��Ķ���ָ��      0x10
wire [ 7:0] sr;   // status register ״̬�Ĵ���              0x14

	// done signal: command completed, clear command register
	wire done;


wire write_enable;
wire read_enable;
wire write_enable0x00;
wire write_enable0x04;
wire write_enable0x08;
wire write_enable0x0C;
wire write_enable0x10;

/*���ݶ�Ӧ��ַ���ɶ�Ӧ�Ĵ����������ź�*/
/*��ַ���������ߵ�byte��ַ����4����Word��ַ*/
assign read_enable = PSEL & (~PWRITE);//��PENABLEʱ��������

assign write_enable0x00 = write_enable &(PADDR[11:2] == 10'h00);
assign write_enable0x04 = write_enable &(PADDR[11:2] == 10'h01);
assign write_enable0x08 = write_enable &(PADDR[11:2] == 10'h02);
assign write_enable0x0C = write_enable &(PADDR[11:2] == 10'h03);
assign write_enable0x10 = write_enable &(PADDR[11:2] == 10'h04);



/*Generate prescale reg*/
always @(posedge  PCLK or negedge PREST_n) begin
  if(PREST_n == 1'b0)
  begin
    prer <= 16'hffff;
  end
  else begin
    if (write_enable0x00) begin
      prer <= {PWDATA[16:0]};
    end
  end
end

/*Generate ctr reg*/
always @(posedge  PCLK or negedge PREST_n) begin
  if(PREST_n == 1'b0)
  begin
    ctr <= 8'h0;
  end
  else begin
    if (write_enable0x04) begin
      ctr <= {PWDATA[7:0]};
    end
  end
end

/*Generate txr reg*/
always @(posedge  PCLK or negedge PREST_n) begin
  if(PREST_n == 1'b0)
  begin
    txr <= 8'h0;
  end
  else begin
    if (write_enable0x08) begin
      txr <= {PWDATA[7:0]};
    end
  end
end

// generate command register (special case)
	always @(posedge PCLK or negedge PREST_n)
	  if (!PREST_n)
	    cr <= #1 8'h0;
	  else if (write_enable0x10)
	    begin
	        if (core_en & (PADDR[4:2] == 3'b100) )
	          cr <= #1 PWDATA[7:0];
	    end
	  else
	    begin
	        if (done | i2c_al)              //�ٲ�ʧ�ܶ�ʧ����Ȩ����I2CͨѶ���
	          cr[7:4] <= #1 4'h0;           // clear command bits when done
	                                        // or when aribitration lost
	        cr[2:1] <= #1 2'b0;             // reserved bits
	        cr[0]   <= #1 1'b0;             // clear IRQ_ACK bit
	    end


/***********************/
/** generate read Dat***/
/***********************/
reg reg_muxread_byte0;
reg reg_muxread_byte1;
reg reg_muxread_byte2;
reg reg_muxread_byte3;
reg reg_read_mux_word;
/*����߼���������Ĵ�����ֵ*/
always @(PADDR or cr or ctr or sr or prer or txr or rxr) begin
  case (PADDR[4:2])
    3'h0:
      begin
        reg_muxread_byte0 <= prer[8:0];
        reg_muxread_byte1 <= prer[15:9];
        reg_muxread_byte2 <= 8'h0;
        reg_muxread_byte3 <= 8'h0;
      end
    3'h1:
      begin
        reg_muxread_byte0 <= ctr;
        reg_muxread_byte1 <= 8'h0;
        reg_muxread_byte2 <= 8'h0;
        reg_muxread_byte3 <= 8'h0;
      end
    3'h2:
      begin
        reg_muxread_byte0 <= txr;
        reg_muxread_byte1 <= 8'h0;
        reg_muxread_byte2 <= 8'h0;
        reg_muxread_byte3 <= 8'h0;
      end
    3'h3:
    begin
      reg_muxread_byte0 <= rxr;
      reg_muxread_byte1 <= 8'h0;
      reg_muxread_byte2 <= 8'h0;
      reg_muxread_byte3 <= 8'h0;
    end
    3'h4:
    begin
      reg_muxread_byte0 <= cr;
      reg_muxread_byte1 <= 8'h0;
      reg_muxread_byte2 <= 8'h0;
      reg_muxread_byte3 <= 8'h0;
    end
    3'h5:
    begin
      reg_muxread_byte0 <= sr;
      reg_muxread_byte1 <= 8'h0;
      reg_muxread_byte2 <= 8'h0;
      reg_muxread_byte3 <= 8'h0;
    end
    default:
      reg_muxread_byte0 <= 8'h0;
      reg_muxread_byte1 <= 8'h0;
      reg_muxread_byte2 <= 8'h0;
      reg_muxread_byte3 <= 8'h0;
  endcase
end
/*tans Read Data To second buf*/
/*�ȵ�ʱ������֮����д�źŵ������������뻺�棬Ȼ��ȴ�read_enable*/
always @(posedge PCLK or negedge PREST_n) begin
  if (PREST_n == 1'b0) begin
    reg_read_mux_word <= 32'h0;
  end else if (read_enable) begin
    reg_read_mux_word <= {reg_muxread_byte3,reg_muxread_byte2,reg_muxread_byte1;reg_muxread_byte0};
  end
  else begin
    reg_read_mux_word <= reg_read_mux_word;
  end
end
/*Ҫ�������������ٱ�������ʱ�����ڵȴ�PENABLE*/
assign PRDATA = (read_enable & PENABLE)?(reg_read_mux_word):(32'h0);

	// decode command register
	wire sta  = cr[7];      //STA, generate (repeated) start condition
	wire sto  = cr[6];      //STO, generate stop condition
	wire rd   = cr[5];      //RD, read from slav
	wire wr   = cr[4];      //WR, write to slave
	wire ack  = cr[3];      //ACK, when a receiver, sent ACK (ACK = ��0��) or NACK (ACK = ��1��)
	wire iack = cr[0];      //IACK, Interrupt acknowledge. When set, clears a pending interrupt.

	// core enable signal
	wire core_en;
  wire ien;
  //decoce control regiseter
	assign core_en = ctr[7];
  assign ien = ctr[6];
  
	// status register signals
	wire irxack;
	reg  rxack;       // received aknowledge from slave
	reg  tip;         // transfer in progress
	reg  irq_flag;    // interrupt pending flag
	wire i2c_busy;    // bus busy (start signal detected)
	wire i2c_al;      // i2c bus arbitration lost
	reg  al;          // status register arbitration lost bit


	// assign status register 
	assign sr[7]   = rxack;
  /*
  RxACK, Received acknowledge from slave.
  This flag represents acknowledge from the addressed slave.
  ��1�� = No acknowledge receive;��0�� = Acknowledge received
  */
	assign sr[6]   = i2c_busy;
	assign sr[5]   = al;
	assign sr[4:2] = 3'h0; // 
	assign sr[1]   = tip;
	assign sr[0]   = irq_flag;
  /*
   This bit is set when an interrupt is pending,
   which will cause a processor interrupt request if the IEN bit is set.
  The Interrupt Flag is set when:one byte transfer has been completed ;
  arbitration is lost*/

	// status register block + interrupt request signal
	always @(posedge PCLK or negedge PREST_n)
	  if (!PREST_n)
	    begin
	        al       <= #1 1'b0;
	        rxack    <= #1 1'b0;
	        tip      <= #1 1'b0;
	        irq_flag <= #1 1'b0;
	    end
	  else
	    begin
	        al       <= #1 i2c_al | (al & ~sta);
	        rxack    <= #1 irxack;
	        tip      <= #1 (rd | wr);
	        irq_flag <= #1 (done | i2c_al | irq_flag) & ~iack; // interrupt request flag is always generated
          /*���Լ���һ�±�ʾ����״ֱ̬������������Լ�*/
      end

// generate interrupt request signals
	always @(posedge PCLK or negedge PREST_n)
	  if (!PREST_n)
	    inta_o <= #1 1'b0;
	  else
	    inta_o <= #1 irq_flag && ien; // interrupt signal is only generated when IEN (interrupt enable bit is set)


endmodule
