//
//
//
//
//

module bk_test_2ch(
  input wire i_clk_25m,  //25m时钟输入
  output reg o_bk_pulse0,
output reg o_bk_pulse1,
output reg o_bk_pulse2,
output reg o_bk_pulse3,
output reg o_bk_pulse4,
output reg o_bk_pulse5  //反馈信号
);
reg [18:0]cnt = 18'd0;  //累加计数
always@(posedge i_clk_25m) begin
  cnt <= cnt + 1'd1;
  if(cnt <= 18'd875) begin	
	o_bk_pulse0 <= 1'b1; 
	o_bk_pulse1 <= 1'b1; 
/*	o_bk_pulse2 <= 1'b1; 
   o_bk_pulse3 <= 1'b1; 
   o_bk_pulse4 <= 1'b1; 
   o_bk_pulse5 <= 1'b0; */	
  end
  else if(cnt <= 18'd250000) begin	
	o_bk_pulse0 <= 1'b0; 
	o_bk_pulse1 <= 1'b0; 
/*	o_bk_pulse2 <= 1'b0; 
   o_bk_pulse3 <= 1'b0; 
   o_bk_pulse4 <= 1'b0; 
   o_bk_pulse5 <= 1'b0; */		
  end  
  else if(cnt <= 18'd250875) begin	
	o_bk_pulse0 <= 1'b0; 
	o_bk_pulse1 <= 1'b1; 
/*	o_bk_pulse2 <= 1'b1; 
   o_bk_pulse3 <= 1'b1; 
   o_bk_pulse4 <= 1'b1; 
   o_bk_pulse5 <= 1'b1;*/	 	
  end
  else if(cnt <= 18'd500000) begin	
	o_bk_pulse0 <= 1'b0; 
	o_bk_pulse1 <= 1'b0; 
/*	o_bk_pulse2 <= 1'b0; 
   o_bk_pulse3 <= 1'b0; 
   o_bk_pulse4 <= 1'b0; 
   o_bk_pulse5 <= 1'b0; */		
  end  
  else begin
    cnt <= 18'd0;
    o_bk_pulse0 <= 1'b0; 
	 o_bk_pulse1 <= 1'b0; 
/*	 o_bk_pulse2 <= 1'b0; 
    o_bk_pulse3 <= 1'b0; 
    o_bk_pulse4 <= 1'b0; 
    o_bk_pulse5 <= 1'b0; */	
  end
end
endmodule
