/*功能：dual_port_ram测试，对应iCore3 REVB4.20200814核心板
 *设计：cay
 *日期：2023-4-10
 *说明：
 *    1.双口ram输出击穿结果

 */
 //--------------------------Module_dual_port_ram---------------------------//	
module dual_port_ram(
							input CLK_25M,
							input WR,
							input RD,
							input CS0,
							inout [15:0]DB,
							input [24:16]A,
							output FPGA_LEDR,
							output FPGA_LEDG,
							output FPGA_LEDB
							);
		
//--------------------------pll---------------------------//
/*例化锁相环模块产生需要的时钟*/	
	my_pll    u1(
					 .inclk0(CLK_25M),
					 .c0(PLL_100M)
					);
//--------------------------ram---------------------------//
/*例化双口ram模块*/
	wire [15:0]b_dataout;
	wire [15:0]a_dataout;
	
	my_ram    u2(
					.address_a(a_addr),
					.address_b(A),
					.data_a(a_datain),
					.data_b(DB),
					.clock_a(!a_clk),
					.clock_b(b_clk),
					.rden_a(a_rden),
					.rden_b(!rd),
					.wren_a(a_wren),
					.wren_b(1'd0),
					.q_a(a_dataout),
					.q_b(b_dataout)
					);
//--------------------------rst_n---------------------------//
/*产生复位信号*/
	reg [3:0]cnt_rst = 4'd0;
	
	always @ (posedge CLK_25M)
		if(cnt_rst == 4'd10)
			begin
				cnt_rst <= 4'd10;
			end
		else
			cnt_rst <= cnt_rst + 4'd1;
			
	wire rst_n = (cnt_rst == 4'd10) ? 1'd1 : 1'd0;
	
//--------------------------delay---------------------------//
/*对CLK_25M时钟做延时处理*/
	reg clk1,clk2;
	
	always @(posedge PLL_100M or negedge rst_n) 
		begin
			if(!rst_n)
				begin 
					clk1 <= 1'd0;
					clk2 <= 1'd0;
				end 
			else
				{clk2,clk1} <= {clk1,CLK_25M};
		end
		
	wire a_clk = (CLK_25M & clk1);	
	
//--------------------------cnt----------------------------//
/*ram地址数据计数器*/
	reg [9:0]cnt; 
	
	always @ (posedge CLK_25M or negedge rst_n)
		begin
			if(!rst_n)
			   begin
					cnt <= 10'd0;
			   end 
			else if(cnt == 10'd511)
				cnt <= 10'd0;
			else 
				cnt <= cnt + 10'd1;	
		end
		
//--------------------------ram_a wr&rd---------------------------//
/*实现ram_a端口的写与读功能，计数器0到255为写，256到511为读*/
  reg a_wren,a_rden;
  reg [9:0]a_datain;
  reg [9:0]a_addr;
  
	always @ (posedge a_clk or negedge rst_n)
		begin
			if(!rst_n)
			   begin
					a_wren <= 1'd0;
					a_rden <= 1'd0;
					a_datain <= 10'd0;
					a_addr <= 10'd0;
			   end 
			else if(cnt >= 10'd0 && cnt <=10'd255)
				begin
					a_wren <= 1'd1;                        //写使能信号拉高
					a_addr <= cnt;									//写地址
					a_datain <= cnt;								//写入数据
					a_rden <= 1'd0;								//读使能信号拉低
				end 
			else if(cnt >= 10'd256 && cnt <= 10'd511)
				begin
					a_rden <= 1'd1;								//读使能信号拉高
					a_addr <= cnt - 10'd256;					//读地址
					a_wren <= 1'd0;								//写使能信号拉低
				end
		end
		
//--------------------------ram_a_error---------------------------//	
/*错误判断标志，读取的地址和数据相等时标志为0，不相等时为1*/
	reg error;
	
	always @ ( negedge clk1 or negedge rst_n)
		begin
			if(!rst_n)
				begin
					error <= 1'd0;		
				end
			else 
				begin
					if(a_wren || a_dataout == a_addr)      //判断读取地址和数据是否相等
							error <= 1'd0;
					else 
							error <= 1'd1;	
				end		
		end	
		
//--------------------------ram_a_led---------------------------//
/*根据错误标志控制led灯的颜色，无误时绿灯亮，有误时红灯亮*/
	reg ledr,ledg,ledb;		
	always @(posedge error or negedge rst_n)
		if(!rst_n)
			begin
				ledr <= 1'd1;
				ledg <= 1'd0;
				ledb <= 1'd1;		
			end
		else 
			begin
				ledr <= 1'd0;
				ledg <= 1'd1;
				ledb <= 1'd1;
			end
	
	assign {FPGA_LEDR,FPGA_LEDG,FPGA_LEDB} = {ledr,ledg,ledb};

//--------------------------ram_b_rd---------------------------//
/*控制ram_b端口的读取功能，当读信号来临时，读取相应的数据发送至fsmc总线*/
	wire rd = (CS0 | RD);		 //提取读信号
	wire wr = (CS0 | WR);       //提取写信号
	
	reg wr_clk1,wr_clk2;
	always @(posedge PLL_100M or negedge rst_n)
		begin
			if(!rst_n)
				begin
					wr_clk1 <= 1'd1;
					wr_clk2 <= 1'd1;
				end
			else
				{wr_clk2,wr_clk1} <= {wr_clk1,wr};	
		end

	wire b_clk = (!wr_clk2 | !rd);							//提取ram_b端口的时钟
	assign DB = !rd ? b_dataout : 16'hzzzz;				//读信号来临，读取ram_b端口数据
	
//--------------------------endmodule---------------------------//
endmodule	