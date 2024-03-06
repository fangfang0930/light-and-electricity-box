/*功能：CHK LE Board 击穿检测，对应iCore3 REVB4.20200814核心板
 *设计：cay
 *日期：2023-4-10
 *说明：
 *    1.输出为1，则输入光头亮，
      2.检测击穿和BOD
 */ 

module	counter_6bits (

input	wire	  i_clk_50m,         //50m晶振
input	wire    i_signal,          //击穿信号
input	wire    i_signal_forward,  //正相触发脉冲
input	wire    i_signal_negative, //负相触发脉冲
input	wire    i_signal_forbid,   //脉冲禁止信号，1禁止

output	reg		o_SCR_forward_state,   //正相击穿信号，1表示动作
output	reg		o_SCR_negative_state,  //负相击穿信号，1表示动作

output	reg		o_SCR_forward_BOD,   //正相BOD信号，1表示动作
output	reg		o_SCR_negative_BOD  //负相BOD信号，1表示动作

//output	reg		o_SCR_forward_power,
//output	reg		o_SCR_negative_power,

//output	reg		o_SCR_forward_bank,
//output	reg		o_SCR_negative_bank
);

// 500us，触发脉冲发出后，在脉冲停止位之前，有10us脉冲信号，则BOD动作
parameter	pulse_check_time1=17'd25000;//hff-17'd25000;
// 848us
parameter	pulse_check_time2=17'd42400;//17'd42400;
// 1048us
parameter	pulse_check_time3=17'd52400;//17'd52400;
// 10000us,正脉冲检负相击穿信号，负脉冲检正相击穿信号
//parameter	pulse_check_time4=20'd500000;
parameter	pulse_check_time4=20'd900000;//20'd900000;

reg[20:0] counter_forward=20'd0;
reg[20:0] counter_negative=20'd0;

wire   f_signal;

in_shake_scr_pulse inshake_sig(
   .clk(i_clk_50m),
   .in_i(i_signal),
   .in_o(f_signal)
);

reg i_signal_forward_buff=1'b0;
reg i_signal_negative_buff=1'b0;
reg f_signal_buff1=1'b0;
reg f_signal_buff2=1'b0;

reg signal_forward_flag=1'b0;
reg signal_negative_flag=1'b0;

reg[1:0] code_forward=2'd0;
reg[1:0] code_negative=2'd0;

always@(posedge i_clk_50m)
begin
    if(i_signal_forbid == 1'b1)
    begin
		o_SCR_forward_state<=1'b1;
//      o_SCR_negative_state<=1'b1;
		o_SCR_forward_BOD<=1'b1;
//		o_SCR_forward_power<=1'b1;
//		o_SCR_forward_bank<=1'b1;
		counter_forward<=20'd0;
    end
    else
    begin       
//        o_SCR_forward_bank<=1'b0;
    
		if((i_signal_forward_buff==1'b0)&&(i_signal_forward==1'b1))
		begin
			counter_forward<=20'd1;
			i_signal_forward_buff<=i_signal_forward;
		end
		else if(counter_forward>=20'd1)
		begin			
			if(counter_forward>=pulse_check_time4+20'd5)
				counter_forward<=20'd0;
			else
				counter_forward<=counter_forward+20'd1;
				
			i_signal_forward_buff<=i_signal_forward;
		end
		else
			i_signal_forward_buff<=i_signal_forward;
		
		if((counter_forward<=pulse_check_time1)&&(counter_forward > 20'd0))
		begin
			if((f_signal_buff1==1'b0)&&(f_signal==1'b1))
			begin
				code_forward[1]<=1'b1;
				signal_forward_flag<=1'b1;
			end
			else if(signal_forward_flag==1'b0)
				code_forward[1]<=1'b0;
			
			f_signal_buff1<=f_signal;			
		end
		else if(counter_forward<=pulse_check_time2)
		begin
			signal_forward_flag<=1'b0;
			f_signal_buff1<=f_signal;
        end
		else if(counter_forward<=pulse_check_time4)
		begin
			if((f_signal_buff1==1'b0)&&(f_signal==1'b1))
			begin
				code_forward[0]<=1'b0;
				signal_forward_flag<=1'b1;
			end
			else if(signal_forward_flag==1'b0)
				code_forward[0]<=1'b1;
			f_signal_buff1<=f_signal;
		end
		else if(counter_forward==pulse_check_time4+20'd2)
		begin
			signal_forward_flag<=1'b0;
			f_signal_buff1<=f_signal;
			
	/*		if(code_forward>2'd0)
				o_SCR_forward_state<=1'b1;
			else if(code_forward==2'd0)
				o_SCR_forward_state<=1'b0;		*/	
				
		   o_SCR_forward_state<=code_negative[0];	 		    
			o_SCR_forward_BOD<=code_forward[1];												    
//			o_SCR_forward_power<=code_forward[0];				    
		end
		else
			f_signal_buff1<=f_signal;			   
    end  
end

always@(posedge i_clk_50m)
begin
    if(i_signal_forbid == 1'b1)
    begin
		o_SCR_negative_state<=1'b1;
//      o_SCR_forward_state<=1'b1;
		o_SCR_negative_BOD<=1'b1;
//		o_SCR_negative_power<=1'b1;
//		o_SCR_negative_bank<=1'b1;
		counter_negative<=20'd0;
    end
    else
    begin		
//		o_SCR_negative_bank<=1'b0;
		
		if((i_signal_negative_buff==1'b0)&&(i_signal_negative==1'b1))
		begin
			counter_negative<=20'd1;
		end
		else if(counter_negative>=20'd1)
		begin			
			if(counter_negative>=pulse_check_time4+20'd5)
				counter_negative<=20'd0;
			else
				counter_negative<=counter_negative+20'd1;
		end
		
		if((counter_negative<=pulse_check_time1)&&(counter_negative > 20'd0))
		begin
			if((f_signal_buff2==1'b0)&&(f_signal==1'b1))
			begin
				code_negative[1]<=1'b1;
				signal_negative_flag<=1'b1;
			end
			else if(signal_negative_flag==1'b0)
				code_negative[1]<=1'b0;	
				
			f_signal_buff2<=f_signal;
					
		end
		else if(counter_negative<=pulse_check_time2)
		begin
			signal_negative_flag<=1'b0;
			f_signal_buff2<=f_signal;
        end
		else if(counter_negative<=pulse_check_time4)
		begin
			if((f_signal_buff2==1'b0)&&(f_signal==1'b1))
			begin
				code_negative[0]<=1'b0;
				signal_negative_flag<=1'b1;
			end
			else if(signal_negative_flag==1'b0)
				code_negative[0]<=1'b1;
			f_signal_buff2<=f_signal;
		end
		else if(counter_negative==pulse_check_time4+20'd2)
		begin
			signal_negative_flag<=1'b0;
			f_signal_buff2<=f_signal;
			
		/*	if(code_negative>2'd0)
				o_SCR_negative_state<=1'b1;	
			else if(code_negative==2'd0)
				o_SCR_negative_state<=1'b0;	*/	
         o_SCR_negative_state<=code_forward[0];				
		    
			o_SCR_negative_BOD<=code_negative[1];												    
//			o_SCR_negative_power<=code_negative[0];											    		    
		end
		else
			f_signal_buff2<=f_signal;
		
		i_signal_negative_buff<=i_signal_negative;
			   		
    end  
end


endmodule

