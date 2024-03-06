/*功能：CHK LE Board io测试
 *设计：cay
 *日期：2023-4-4
 *
 */

module in_shake(
input	wire	clk,
input	wire	in_i,
output	reg		in_o

);

reg in_i_buff1=1'b0;
reg in_i_buff2=1'b0;

always @(posedge clk )
begin
	in_i_buff1<=in_i;
	in_i_buff2<=in_i_buff1;
end

parameter io_shake=6'd50;
reg[5:0]	c_num;
reg[5:0]	o_num;

always @(posedge clk )
if(in_i_buff2)
	begin
	o_num<=6'd0;
	c_num<=c_num+6'd1;
	if(c_num>=io_shake)
	begin
		c_num<=io_shake;
		in_o<=1'b1;
	end
end
else
begin
	c_num<=6'd0;
	o_num<=o_num+6'd1;
	if(o_num>=io_shake)
	begin
		o_num<=io_shake;
		in_o<=1'b0;
	end
end


endmodule
