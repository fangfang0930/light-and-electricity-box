//IO输入防抖模块
//2010-08-20
//by wx
/*
对输入进行防抖处理，避免误动
防抖计数
io_shake:25
*/


module in_shake_scr_pulse(
input	wire	clk,
input	wire	in_i,
output	reg		in_o

);

parameter io_shake=10'd10;
reg[9:0]	c_num;
reg[9:0]	o_num;

reg in_i_buff1=1'b0;
reg in_i_buff2=1'b0;

always @(posedge clk )
begin
	in_i_buff1<=in_i;
	in_i_buff2<=in_i_buff1;
end



always @(posedge clk )
if(in_i_buff2)
	begin
	o_num<=10'd0;
	c_num<=c_num+10'd1;
	if(c_num>=io_shake)
	begin
		c_num<=io_shake;
		in_o<=1'b1;
	end
end
else
begin
	c_num<=10'd0;
	o_num<=o_num+10'd1;
	if(o_num>=io_shake)
	begin
		o_num<=io_shake;
		in_o<=1'b0;
	end
end


endmodule
