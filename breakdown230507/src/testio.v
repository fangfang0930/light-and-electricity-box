/*功能：CHK LE Board io测试，对应iCore3 REVB4.20200814核心板
 *设计：cay
 *日期：2023-4-4
 *说明：
 *    1.输出设置1'b1，则输出光头亮
      2.5-5-5-5-5触发脉冲
		    5       5       5                            5
		  |---| 5 |---| 5 |---|          830           |---|
		--|   |---|   |---|   |------------------------|   |--
 */
 
module testio(
   input wire i_clk_10m,       //频率10Mhz
   input wire i_clk_50m,       //频率50Mhz
	input wire i_led_key,         //FPGA 按键
	input wire [5:0]i_fiber_ctl,   //主控触发脉冲宽度500us
	input wire [5:0]i_fiber_var,  //用于击穿板检丢脉冲
	input wire [25:0]i_Opt_Receive_State_a,  //A相TE板回馈信号
	input wire [25:0]i_Opt_Receive_State_b,  //B相TE板回馈信号
	input wire [25:0]i_Opt_Receive_State_c,  //C相TE板回馈信号
	
//	input wire [15:0]i_rama_q, //双口RAM_A口数据接收
	
	input wire i_clk_25m,   //板载晶振频率25Mhz
	input wire i_WR,        //双口RAM_A口写
	input wire i_RD,        //双口RAM_A口读
	input wire i_CS0,       //双口RAM_使能
	inout wire [15:0]io_DB,  //双口RAM_16bit数据
	input wire [24:16]i_A,  //双口RAM_地址
	input wire i_clk_100m,   //板载晶振频率100Mhz
	
	output reg [5:0]o_fiber_ctl,  //至主控的回报脉冲
	output reg [5:0]o_fiber_var,  //至阀组的触发脉冲-hff-debug模拟反馈信号
	// LED
	output reg  o_Led_Run,   //绿色LED
	output reg  o_Led_Error, //红色LED
	output reg  o_Led_Other  //蓝色LED
   //双口ram
// output reg[15:0] o_rama_data, //双口RAM_A口数据写入
// output reg[6:0] o_rama_add,   //双口RAM_A口写数据地址
//	output reg o_rama_txrx_en    //双口RAM_A口写数据使能,1:写数据，0:读数据
	//power EN
//	output reg  o_power_En_n=1'b1

   );
//	assign o_fiber = 1'd0;

/*
//运行和故障的LED显示
reg[11:0] counter_led_run=12'd0;
reg[11:0] counter_led_run_buff=12'd0;

always @(posedge i_clk_50m)
begin
   counter_led_run_buff<=counter_led_run;
     
   if(clk_dsp_num >= 10'd100)
      o_power_En_n<=1'd0;
   if(sys_init || sys_fault || b_error)
      begin
      o_Led_Error<=1'b0;
      o_Led_Run<=1'b1;
      end
   else if((counter_led_run==12'd999)&&(counter_led_run_buff==12'd998))
      begin
      o_Led_Error<=1'b1;
      o_Led_Run<=~o_Led_Run;
      end
   else
      o_Led_Error<=1'b1;
end
*/
//信号抗干扰
wire[2:0] Opt_forward_pulse; 
wire[2:0] Opt_negative_pulse;
wire[2:0] Opt_scr_forward_pulse_back;
wire[2:0] Opt_scr_negative_pulse_back;

in_shake pulse1(.clk(i_clk_50m), .in_i(i_fiber_ctl[0]), .in_o(Opt_forward_pulse[0]));   //主控Pab+触发脉冲抗干扰,500us
in_shake pulse2(.clk(i_clk_50m), .in_i(i_fiber_ctl[1]), .in_o(Opt_negative_pulse[0]));  //主控Pab-触发脉冲抗干扰,500us
in_shake pulse3(.clk(i_clk_50m), .in_i(i_fiber_ctl[2]), .in_o(Opt_forward_pulse[1]));   //主控Pbc+触发脉冲抗干扰,500us
in_shake pulse4(.clk(i_clk_50m), .in_i(i_fiber_ctl[3]), .in_o(Opt_negative_pulse[1]));  //主控Pbc-触发脉冲抗干扰,500us
in_shake pulse5(.clk(i_clk_50m), .in_i(i_fiber_ctl[4]), .in_o(Opt_forward_pulse[2]));   //主控Pca+触发脉冲抗干扰,500us
in_shake pulse6(.clk(i_clk_50m), .in_i(i_fiber_ctl[5]), .in_o(Opt_negative_pulse[2]));  //主控Pca-触发脉冲抗干扰,500us

in_shake pulse11(.clk(i_clk_50m), .in_i(i_fiber_var[0]), .in_o(Opt_scr_forward_pulse_back[0]));   //SCR Pab+触发回馈脉冲抗干扰
in_shake pulse12(.clk(i_clk_50m), .in_i(i_fiber_var[1]), .in_o(Opt_scr_negative_pulse_back[0]));  //SCR Pab-触发回馈脉冲抗干扰
in_shake pulse13(.clk(i_clk_50m), .in_i(i_fiber_var[2]), .in_o(Opt_scr_forward_pulse_back[1]));   //SCR Pbc+触发回馈脉冲抗干扰
in_shake pulse14(.clk(i_clk_50m), .in_i(i_fiber_var[3]), .in_o(Opt_scr_negative_pulse_back[1]));  //SCR Pbc-触发回馈脉冲抗干扰
in_shake pulse15(.clk(i_clk_50m), .in_i(i_fiber_var[4]), .in_o(Opt_scr_forward_pulse_back[2]));   //SCR Pca+触发回馈脉冲抗干扰
in_shake pulse16(.clk(i_clk_50m), .in_i(i_fiber_var[5]), .in_o(Opt_scr_negative_pulse_back[2]));  //SCR Pca-触发回馈脉冲抗干扰

reg[2:0] Opt_forward_pulse_buff = 1'b0;
reg[2:0] Opt_negative_pulse_buff = 1'b0;

reg[2:0]  pulse_forward_flag = 1'b0;
reg[2:0]  pulse_negative_flag = 1'b0;

reg[19:0] counter_555_flag[5:0];
//1. 脉冲触发
//ab+ SCR脉冲触发5-5-5

always @(posedge i_clk_50m)
begin
 if((Opt_forward_pulse_buff[0] == 1'b0)&&(Opt_forward_pulse[0]== 1'b1))
	pulse_forward_flag[0]<= 1'b1;
	
 if(pulse_forward_flag[0] == 1'b1)
 begin
    counter_555_flag[0] <= counter_555_flag[0] + 20'd1;
    if(counter_555_flag[0] <= 20'd660000)		
		o_fiber_var[0] <= 1'b0;
	
	else if(counter_555_flag[0] <= 20'd660500)	//10us
		o_fiber_var[0] <= 1'b1;
	else 
	begin

		 o_fiber_var[0] <= 1'b0;
		 counter_555_flag[0] <= 20'd0;
		 pulse_forward_flag[0] <= 1'b0;
	end
 end
 else
    begin
		counter_555_flag[0] <= 20'd0;
		o_fiber_var[0] <=1'b0;
    end
  Opt_forward_pulse_buff[0] <= Opt_forward_pulse[0];
end




//ab- SCR脉冲触发
always @(posedge i_clk_50m)
begin
 if((Opt_negative_pulse_buff[0] == 1'b0)&&(Opt_negative_pulse[0]== 1'b1))
	pulse_negative_flag[0]<= 1'b1;
	
 if(pulse_negative_flag[0] == 1'b1)
// if(pulse_forward_flag[0] == 1'b1)
	begin
    counter_555_flag[1] <= counter_555_flag[1] + 17'd1;
    if(counter_555_flag[1] <= 17'd250)		  o_fiber_var[1] <= 1'b1;
	 else if(counter_555_flag[1] <= 17'd500)	  o_fiber_var[1] <=1'b0;
	 else if(counter_555_flag[1] <= 17'd750)	  o_fiber_var[1] <=1'b1;
	 else if(counter_555_flag[1] <= 17'd1000)  o_fiber_var[1] <=1'b0;
	 else if(counter_555_flag[1] <= 17'd1250)  o_fiber_var[1] <=1'b1;
	 else if(counter_555_flag[1] <= 17'd42750) o_fiber_var[1] <=1'b0;
	 else if(counter_555_flag[1] <= 17'd43000) o_fiber_var[1] <=1'b1;
	 else if(counter_555_flag[1] <= 17'd43001) o_fiber_var[1] <=1'b0;
	 else 
		begin
		 o_fiber_var[1] <= 1'b0;
		 counter_555_flag[1] <= 17'd0;
		 pulse_negative_flag[0] <= 1'b0;
		end
   end
 else
    begin
    counter_555_flag[1] <= 17'd0;
    o_fiber_var[1] <=1'b0;
    end
  Opt_negative_pulse_buff[0] <= Opt_negative_pulse[0];
end

//bc+ SCR脉冲触发5-5-5
always @(posedge i_clk_50m)
begin
 if((Opt_forward_pulse_buff[1] == 1'b0)&&(Opt_forward_pulse[1]== 1'b1))
	pulse_forward_flag[1]<= 1'b1;
	
 if(pulse_forward_flag[1] == 1'b1)
// if(pulse_forward_flag[0] == 1'b1)
	begin
    counter_555_flag[2] <= counter_555_flag[2] + 17'd1;
    if(counter_555_flag[2] <= 17'd250)		  o_fiber_var[2] <= 1'b1;
	 else if(counter_555_flag[2] <= 17'd500)	  o_fiber_var[2] <=1'b0;
	 else if(counter_555_flag[2] <= 17'd750)	  o_fiber_var[2] <=1'b1;
	 else if(counter_555_flag[2] <= 17'd1000)  o_fiber_var[2] <=1'b0;
	 else if(counter_555_flag[2] <= 17'd1250)  o_fiber_var[2] <=1'b1;
	 else if(counter_555_flag[2] <= 17'd42750) o_fiber_var[2] <=1'b0;
	 else if(counter_555_flag[2] <= 17'd43000) o_fiber_var[2] <=1'b1;
	 else if(counter_555_flag[2] <= 17'd43001) o_fiber_var[2] <=1'b0;
	 else 
		begin
		 o_fiber_var[2] <= 1'b0;
		 counter_555_flag[2] <= 17'd0;
		 pulse_forward_flag[1] <= 1'b0;
		end
   end
 else
    begin
    counter_555_flag[2] <= 17'd0;
    o_fiber_var[2] <=1'b0;
    end
  Opt_forward_pulse_buff[1] <= Opt_forward_pulse[1];
end

//bc- SCR脉冲触发5-5-5
always @(posedge i_clk_50m)
begin
 if((Opt_negative_pulse_buff[1] == 1'b0)&&(Opt_negative_pulse[1]== 1'b1))
	pulse_negative_flag[1]<= 1'b1;
	
 if(pulse_negative_flag[1] == 1'b1)
// if(pulse_forward_flag[0] == 1'b1)
	begin
    counter_555_flag[3] <= counter_555_flag[3] + 17'd1;
    if(counter_555_flag[3] <= 17'd250)		  o_fiber_var[3] <= 1'b1;
	 else if(counter_555_flag[3] <= 17'd500)	  o_fiber_var[3] <=1'b0;
	 else if(counter_555_flag[3] <= 17'd750)	  o_fiber_var[3] <=1'b1;
	 else if(counter_555_flag[3] <= 17'd1000)  o_fiber_var[3] <=1'b0;
	 else if(counter_555_flag[3] <= 17'd1250)  o_fiber_var[3] <=1'b1;
	 else if(counter_555_flag[3] <= 17'd42750) o_fiber_var[3] <=1'b0;
	 else if(counter_555_flag[3] <= 17'd43000) o_fiber_var[3] <=1'b1;
	 else if(counter_555_flag[3] <= 17'd43001) o_fiber_var[3] <=1'b0;
	 else 
		begin
		 o_fiber_var[3] <= 1'b0;
		 counter_555_flag[3] <= 17'd0;
		 pulse_negative_flag[1] <= 1'b0;
		end
   end
 else
    begin
    counter_555_flag[3] <= 17'd0;
    o_fiber_var[3] <=1'b0;
    end
  Opt_negative_pulse_buff[1] <= Opt_negative_pulse[1];
end

//ca+ SCR脉冲触发5-5-5
always @(posedge i_clk_50m)
begin
 if((Opt_forward_pulse_buff[2] == 1'b0)&&(Opt_forward_pulse[2]== 1'b1))
	pulse_forward_flag[2]<= 1'b1;
	
 if(pulse_forward_flag[2] == 1'b1)
// if(pulse_forward_flag[0] == 1'b1)
	begin
    counter_555_flag[4] <= counter_555_flag[4] + 17'd1;
    if(counter_555_flag[4] <= 17'd250)		  o_fiber_var[4] <= 1'b1;
	 else if(counter_555_flag[4] <= 17'd500)	  o_fiber_var[4] <=1'b0;
	 else if(counter_555_flag[4] <= 17'd750)	  o_fiber_var[4] <=1'b1;
	 else if(counter_555_flag[4] <= 17'd1000)  o_fiber_var[4] <=1'b0;
	 else if(counter_555_flag[4] <= 17'd1250)  o_fiber_var[4] <=1'b1;
	 else if(counter_555_flag[4] <= 17'd42750) o_fiber_var[4] <=1'b0;
	 else if(counter_555_flag[4] <= 17'd43000) o_fiber_var[4] <=1'b1;
	 else if(counter_555_flag[4] <= 17'd43001) o_fiber_var[4] <=1'b0;
	 else 
		begin
		 o_fiber_var[4] <= 1'b0;
		 counter_555_flag[4] <= 17'd0;
		 pulse_forward_flag[2] <= 1'b0;
		end
   end
 else
    begin
    counter_555_flag[4] <= 17'd0;
    o_fiber_var[4] <=1'b0;
    end
  Opt_forward_pulse_buff[2] <= Opt_forward_pulse[2];
end

//ca- SCR脉冲触发5-5-5
always @(posedge i_clk_50m)
begin
 if((Opt_negative_pulse_buff[2] == 1'b0)&&(Opt_negative_pulse[2]== 1'b1))
	pulse_negative_flag[2]<= 1'b1;
	
 if(pulse_negative_flag[2] == 1'b1)
// if(pulse_forward_flag[0] == 1'b1)
	begin
    counter_555_flag[5] <= counter_555_flag[5] + 17'd1;
    if(counter_555_flag[5] <= 17'd250)		  o_fiber_var[5] <= 1'b1;
	 else if(counter_555_flag[5] <= 17'd500)	  o_fiber_var[5] <=1'b0;
	 else if(counter_555_flag[5] <= 17'd750)	  o_fiber_var[5] <=1'b1;
	 else if(counter_555_flag[5] <= 17'd1000)  o_fiber_var[5] <=1'b0;
	 else if(counter_555_flag[5] <= 17'd1250)  o_fiber_var[5] <=1'b1;
	 else if(counter_555_flag[5] <= 17'd42750) o_fiber_var[5] <=1'b0;
	 else if(counter_555_flag[5] <= 17'd43000) o_fiber_var[5] <=1'b1;
	 else if(counter_555_flag[5] <= 17'd43001) o_fiber_var[5] <=1'b0;
	 else 
		begin
		 o_fiber_var[5] <= 1'b0;
		 counter_555_flag[5] <= 17'd0;
		 pulse_negative_flag[2] <= 1'b0;
		end
   end
 else
    begin
    counter_555_flag[5] <= 17'd0;
    o_fiber_var[5] <=1'b0;
    end
  Opt_negative_pulse_buff[2] <= Opt_negative_pulse[2];
end
/*
//主控触发回报脉冲
always @(posedge i_clk_50m)
begin
	o_fiber_ctl[0] <= i_fiber_ctl[0];
	o_fiber_ctl[1] <= i_fiber_ctl[1];
	o_fiber_ctl[2] <= i_fiber_ctl[2];
	o_fiber_ctl[3] <= i_fiber_ctl[3];
	o_fiber_ctl[4] <= i_fiber_ctl[4];
	o_fiber_ctl[5] <= i_fiber_ctl[5];
end*/
/*
//模拟触发脉冲输出
reg[31:0] syn_cnt = 32'd0;

always @(posedge i_clk_50m)
begin
  syn_cnt <= syn_cnt + 1'd1;
  if(syn_cnt == 32'd0) o_fiber_ctl[0] <= 1'b1;
  else if(syn_cnt == 32'd25000) o_fiber_ctl[0] <= 1'b0; 
//  else if(syn_cnt == 32'd500000) o_fiber_ctl[1] <= 1'b1;
//  else if(syn_cnt == 32'd525000) o_fiber_ctl[1] <= 1'b0;
  else if(syn_cnt >= 32'd1000000) syn_cnt <= 32'd0; 
end
*/
reg pulse_forbid_state;
reg Opt_forward_pulse_buff1=1'b0;
reg Opt_negative_pulse_buff1=1'b0;
reg[22:0] counter_forward_pulse_60ms=23'd0;
reg[22:0] counter_negative_pulse_60ms=23'd0;

always @(posedge i_clk_50m)
begin	
	Opt_forward_pulse_buff1<=Opt_forward_pulse[0];
	Opt_negative_pulse_buff1<=Opt_negative_pulse[0];
	
	if((Opt_forward_pulse_buff1==1'b0)&&(Opt_forward_pulse[0]==1'b1))
	begin
		counter_forward_pulse_60ms<=23'd0;
	end
    else
    begin
		if(counter_forward_pulse_60ms>=23'd3000000)
			counter_forward_pulse_60ms<=23'd3000000;
		else
			counter_forward_pulse_60ms<=counter_forward_pulse_60ms+23'd1;
	end
	
	if((Opt_negative_pulse_buff1==1'b0)&&(Opt_negative_pulse[0]==1'b1))
	begin
		counter_negative_pulse_60ms<=23'd0;
	end
    else
    begin
		if(counter_negative_pulse_60ms>=23'd3000000)
			counter_negative_pulse_60ms<=23'd3000000;
		else
			counter_negative_pulse_60ms<=counter_negative_pulse_60ms+23'd1;
	end
end


always @(posedge i_clk_50m)
begin
	if((counter_forward_pulse_60ms < 23'd3000000)&&(counter_negative_pulse_60ms < 23'd3000000))
		pulse_forbid_state<=1'b0;
	else
		pulse_forbid_state<=1'b1;
end










//2. 光电击穿和BOD检测

//A相光电击穿检测，24路信号

//wire [25:0]SCR_forward_stateA;
wire [25:0]SCR_forward_stateA;
wire [25:0]SCR_negative_stateA;
wire [25:0]SCR_forward_BODA;
wire [25:0]SCR_negative_BODA;

counter_6bits OPTA1(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[0]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[0]),
	.o_SCR_negative_state(SCR_negative_stateA[0]),
	.o_SCR_forward_BOD(SCR_forward_BODA[0]),
	.o_SCR_negative_BOD(SCR_negative_BODA[0])
);
counter_6bits OPTA2(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[1]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[1]),
	.o_SCR_negative_state(SCR_negative_stateA[1]),
	.o_SCR_forward_BOD(SCR_forward_BODA[1]),
	.o_SCR_negative_BOD(SCR_negative_BODA[1])
);
counter_6bits OPTA3(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[2]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[2]),
	.o_SCR_negative_state(SCR_negative_stateA[2]),
	.o_SCR_forward_BOD(SCR_forward_BODA[2]),
	.o_SCR_negative_BOD(SCR_negative_BODA[2])
);
counter_6bits OPTA4(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[3]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[3]),
	.o_SCR_negative_state(SCR_negative_stateA[3]),
	.o_SCR_forward_BOD(SCR_forward_BODA[3]),
	.o_SCR_negative_BOD(SCR_negative_BODA[3])
);
counter_6bits OPTA5(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[4]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[4]),
	.o_SCR_negative_state(SCR_negative_stateA[4]),
	.o_SCR_forward_BOD(SCR_forward_BODA[4]),
	.o_SCR_negative_BOD(SCR_negative_BODA[4])
);
counter_6bits OPTA6(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[5]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[5]),
	.o_SCR_negative_state(SCR_negative_stateA[5]),
	.o_SCR_forward_BOD(SCR_forward_BODA[5]),
	.o_SCR_negative_BOD(SCR_negative_BODA[5])
);
counter_6bits OPTA7(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[6]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[6]),
	.o_SCR_negative_state(SCR_negative_stateA[6]),
	.o_SCR_forward_BOD(SCR_forward_BODA[6]),
	.o_SCR_negative_BOD(SCR_negative_BODA[6])
);
counter_6bits OPTA8(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[7]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[7]),
	.o_SCR_negative_state(SCR_negative_stateA[7]),
	.o_SCR_forward_BOD(SCR_forward_BODA[7]),
	.o_SCR_negative_BOD(SCR_negative_BODA[7])
);
counter_6bits OPTA9(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[8]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[8]),
	.o_SCR_negative_state(SCR_negative_stateA[8]),
	.o_SCR_forward_BOD(SCR_forward_BODA[8]),
	.o_SCR_negative_BOD(SCR_negative_BODA[8])
);
counter_6bits OPTA10(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[9]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[9]),
	.o_SCR_negative_state(SCR_negative_stateA[9]),
	.o_SCR_forward_BOD(SCR_forward_BODA[9]),
	.o_SCR_negative_BOD(SCR_negative_BODA[9])
);

counter_6bits OPTA11(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[10]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[10]),
	.o_SCR_negative_state(SCR_negative_stateA[10]),
	.o_SCR_forward_BOD(SCR_forward_BODA[10]),
	.o_SCR_negative_BOD(SCR_negative_BODA[10])
);

counter_6bits OPTA12(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[11]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[11]),
	.o_SCR_negative_state(SCR_negative_stateA[11]),
	.o_SCR_forward_BOD(SCR_forward_BODA[11]),
	.o_SCR_negative_BOD(SCR_negative_BODA[11])
);

counter_6bits OPTA13(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[12]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[12]),
	.o_SCR_negative_state(SCR_negative_stateA[12]),
	.o_SCR_forward_BOD(SCR_forward_BODA[12]),
	.o_SCR_negative_BOD(SCR_negative_BODA[12])
);

counter_6bits OPTA14(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[13]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[13]),
	.o_SCR_negative_state(SCR_negative_stateA[13]),
	.o_SCR_forward_BOD(SCR_forward_BODA[13]),
	.o_SCR_negative_BOD(SCR_negative_BODA[13])
);

counter_6bits OPTA15(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[14]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[14]),
	.o_SCR_negative_state(SCR_negative_stateA[14]),
	.o_SCR_forward_BOD(SCR_forward_BODA[14]),
	.o_SCR_negative_BOD(SCR_negative_BODA[14])
);
counter_6bits OPTA16(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[15]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[15]),
	.o_SCR_negative_state(SCR_negative_stateA[15]),
	.o_SCR_forward_BOD(SCR_forward_BODA[15]),
	.o_SCR_negative_BOD(SCR_negative_BODA[15])
);
counter_6bits OPTA17(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[16]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[16]),
	.o_SCR_negative_state(SCR_negative_stateA[16]),
	.o_SCR_forward_BOD(SCR_forward_BODA[16]),
	.o_SCR_negative_BOD(SCR_negative_BODA[16])
);
counter_6bits OPTA18(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[17]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[17]),
	.o_SCR_negative_state(SCR_negative_stateA[17]),
	.o_SCR_forward_BOD(SCR_forward_BODA[17]),
	.o_SCR_negative_BOD(SCR_negative_BODA[17])
);
counter_6bits OPTA19(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[18]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[18]),
	.o_SCR_negative_state(SCR_negative_stateA[18]),
	.o_SCR_forward_BOD(SCR_forward_BODA[18]),
	.o_SCR_negative_BOD(SCR_negative_BODA[18])
);
counter_6bits OPTA20(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[19]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[19]),
	.o_SCR_negative_state(SCR_negative_stateA[19]),
	.o_SCR_forward_BOD(SCR_forward_BODA[19]),
	.o_SCR_negative_BOD(SCR_negative_BODA[19])
);
counter_6bits OPTA21(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[20]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[20]),
	.o_SCR_negative_state(SCR_negative_stateA[20]),
	.o_SCR_forward_BOD(SCR_forward_BODA[20]),
	.o_SCR_negative_BOD(SCR_negative_BODA[20])
);
counter_6bits OPTA22(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[21]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[21]),
	.o_SCR_negative_state(SCR_negative_stateA[21]),
	.o_SCR_forward_BOD(SCR_forward_BODA[21]),
	.o_SCR_negative_BOD(SCR_negative_BODA[21])
);
counter_6bits OPTA23(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[22]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[22]),
	.o_SCR_negative_state(SCR_negative_stateA[22]),
	.o_SCR_forward_BOD(SCR_forward_BODA[22]),
	.o_SCR_negative_BOD(SCR_negative_BODA[22])
);
counter_6bits OPTA24(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[23]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[23]),
	.o_SCR_negative_state(SCR_negative_stateA[23]),
	.o_SCR_forward_BOD(SCR_forward_BODA[23]),
	.o_SCR_negative_BOD(SCR_negative_BODA[23])
);
counter_6bits OPTA25(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[24]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[24]),
	.o_SCR_negative_state(SCR_negative_stateA[24]),
	.o_SCR_forward_BOD(SCR_forward_BODA[24]),
	.o_SCR_negative_BOD(SCR_negative_BODA[24])
);
counter_6bits OPTA26(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_a[25]),
	.i_signal_forward(Opt_forward_pulse[0]),
	.i_signal_negative(Opt_negative_pulse[0]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateA[25]),
	.o_SCR_negative_state(SCR_negative_stateA[25]),
	.o_SCR_forward_BOD(SCR_forward_BODA[25]),
	.o_SCR_negative_BOD(SCR_negative_BODA[25])
);
//B相光电击穿检测，24路信号

wire [25:0] SCR_forward_stateB;
wire [25:0] SCR_negative_stateB;
wire [25:0] SCR_forward_BODB;
wire [25:0] SCR_negative_BODB;

counter_6bits OPTB1(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[0]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[0]),
	.o_SCR_negative_state(SCR_negative_stateB[0]),
	.o_SCR_forward_BOD(SCR_forward_BODB[0]),
	.o_SCR_negative_BOD(SCR_negative_BODB[0])
);
counter_6bits OPTB2(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[1]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[1]),
	.o_SCR_negative_state(SCR_negative_stateB[1]),
	.o_SCR_forward_BOD(SCR_forward_BODB[1]),
	.o_SCR_negative_BOD(SCR_negative_BODB[1])
);
counter_6bits OPTB3(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[2]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[2]),
	.o_SCR_negative_state(SCR_negative_stateB[2]),
	.o_SCR_forward_BOD(SCR_forward_BODB[2]),
	.o_SCR_negative_BOD(SCR_negative_BODB[2])
);
counter_6bits OPTB4(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[3]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[3]),
	.o_SCR_negative_state(SCR_negative_stateB[3]),
	.o_SCR_forward_BOD(SCR_forward_BODB[3]),
	.o_SCR_negative_BOD(SCR_negative_BODB[3])
);
counter_6bits OPTB5(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[4]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[4]),
	.o_SCR_negative_state(SCR_negative_stateB[4]),
	.o_SCR_forward_BOD(SCR_forward_BODB[4]),
	.o_SCR_negative_BOD(SCR_negative_BODB[4])
);
counter_6bits OPTB6(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[5]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[5]),
	.o_SCR_negative_state(SCR_negative_stateB[5]),
	.o_SCR_forward_BOD(SCR_forward_BODB[5]),
	.o_SCR_negative_BOD(SCR_negative_BODB[5])
);
counter_6bits OPTB7(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[6]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[6]),
	.o_SCR_negative_state(SCR_negative_stateB[6]),
	.o_SCR_forward_BOD(SCR_forward_BODB[6]),
	.o_SCR_negative_BOD(SCR_negative_BODB[6])
);
counter_6bits OPTB8(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[7]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[7]),
	.o_SCR_negative_state(SCR_negative_stateB[7]),
	.o_SCR_forward_BOD(SCR_forward_BODB[7]),
	.o_SCR_negative_BOD(SCR_negative_BODB[7])
);
counter_6bits OPTB9(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[8]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[8]),
	.o_SCR_negative_state(SCR_negative_stateB[8]),
	.o_SCR_forward_BOD(SCR_forward_BODB[8]),
	.o_SCR_negative_BOD(SCR_negative_BODB[8])
);
counter_6bits OPTB10(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[9]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[9]),
	.o_SCR_negative_state(SCR_negative_stateB[9]),
	.o_SCR_forward_BOD(SCR_forward_BODB[9]),
	.o_SCR_negative_BOD(SCR_negative_BODB[9])
);
counter_6bits OPTB11(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[10]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[10]),
	.o_SCR_negative_state(SCR_negative_stateB[10]),
	.o_SCR_forward_BOD(SCR_forward_BODB[10]),
	.o_SCR_negative_BOD(SCR_negative_BODB[10])
);
counter_6bits OPTB12(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[11]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[11]),
	.o_SCR_negative_state(SCR_negative_stateB[11]),
	.o_SCR_forward_BOD(SCR_forward_BODB[11]),
	.o_SCR_negative_BOD(SCR_negative_BODB[11])
);
counter_6bits OPTB13(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[12]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[12]),
	.o_SCR_negative_state(SCR_negative_stateB[12]),
	.o_SCR_forward_BOD(SCR_forward_BODB[12]),
	.o_SCR_negative_BOD(SCR_negative_BODB[12])
);
counter_6bits OPTB14(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[13]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[13]),
	.o_SCR_negative_state(SCR_negative_stateB[13]),
	.o_SCR_forward_BOD(SCR_forward_BODB[13]),
	.o_SCR_negative_BOD(SCR_negative_BODB[13])
);
counter_6bits OPTB15(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[14]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[14]),
	.o_SCR_negative_state(SCR_negative_stateB[14]),
	.o_SCR_forward_BOD(SCR_forward_BODB[14]),
	.o_SCR_negative_BOD(SCR_negative_BODB[14])
);
counter_6bits OPTB16(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[15]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[15]),
	.o_SCR_negative_state(SCR_negative_stateB[15]),
	.o_SCR_forward_BOD(SCR_forward_BODB[15]),
	.o_SCR_negative_BOD(SCR_negative_BODB[15])
);
counter_6bits OPTB17(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[16]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[16]),
	.o_SCR_negative_state(SCR_negative_stateB[16]),
	.o_SCR_forward_BOD(SCR_forward_BODB[16]),
	.o_SCR_negative_BOD(SCR_negative_BODB[16])
);
counter_6bits OPTB18(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[17]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[17]),
	.o_SCR_negative_state(SCR_negative_stateB[17]),
	.o_SCR_forward_BOD(SCR_forward_BODB[17]),
	.o_SCR_negative_BOD(SCR_negative_BODB[17])
);
counter_6bits OPTB19(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[18]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[18]),
	.o_SCR_negative_state(SCR_negative_stateB[18]),
	.o_SCR_forward_BOD(SCR_forward_BODB[18]),
	.o_SCR_negative_BOD(SCR_negative_BODB[18])
);
counter_6bits OPTB20(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[19]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[19]),
	.o_SCR_negative_state(SCR_negative_stateB[19]),
	.o_SCR_forward_BOD(SCR_forward_BODB[19]),
	.o_SCR_negative_BOD(SCR_negative_BODB[19])
);
counter_6bits OPTB21(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[20]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[20]),
	.o_SCR_negative_state(SCR_negative_stateB[20]),
	.o_SCR_forward_BOD(SCR_forward_BODB[20]),
	.o_SCR_negative_BOD(SCR_negative_BODB[20])
);
counter_6bits OPTB22(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[21]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[21]),
	.o_SCR_negative_state(SCR_negative_stateB[21]),
	.o_SCR_forward_BOD(SCR_forward_BODB[21]),
	.o_SCR_negative_BOD(SCR_negative_BODB[21])
);
counter_6bits OPTB23(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[22]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[22]),
	.o_SCR_negative_state(SCR_negative_stateB[22]),
	.o_SCR_forward_BOD(SCR_forward_BODB[22]),
	.o_SCR_negative_BOD(SCR_negative_BODB[22])
);
counter_6bits OPTB24(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[23]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[23]),
	.o_SCR_negative_state(SCR_negative_stateB[23]),
	.o_SCR_forward_BOD(SCR_forward_BODB[23]),
	.o_SCR_negative_BOD(SCR_negative_BODB[23])
);
counter_6bits OPTB25(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[24]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[24]),
	.o_SCR_negative_state(SCR_negative_stateB[24]),
	.o_SCR_forward_BOD(SCR_forward_BODB[24]),
	.o_SCR_negative_BOD(SCR_negative_BODB[24])
);
counter_6bits OPTB26(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_b[25]),
	.i_signal_forward(Opt_forward_pulse[1]),
	.i_signal_negative(Opt_negative_pulse[1]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateB[25]),
	.o_SCR_negative_state(SCR_negative_stateB[25]),
	.o_SCR_forward_BOD(SCR_forward_BODB[25]),
	.o_SCR_negative_BOD(SCR_negative_BODB[25])
);
//C相光电击穿检测，24路信号

wire [25:0] SCR_forward_stateC;
wire [25:0] SCR_negative_stateC;
wire [25:0] SCR_forward_BODC;
wire [25:0] SCR_negative_BODC;

counter_6bits OPTC1(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[0]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[0]),
	.o_SCR_negative_state(SCR_negative_stateC[0]),
	.o_SCR_forward_BOD(SCR_forward_BODC[0]),
	.o_SCR_negative_BOD(SCR_negative_BODC[0])
);
counter_6bits OPTC2(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[1]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[1]),
	.o_SCR_negative_state(SCR_negative_stateC[1]),
	.o_SCR_forward_BOD(SCR_forward_BODC[1]),
	.o_SCR_negative_BOD(SCR_negative_BODC[1])
);
counter_6bits OPTC3(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[2]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[2]),
	.o_SCR_negative_state(SCR_negative_stateC[2]),
	.o_SCR_forward_BOD(SCR_forward_BODC[2]),
	.o_SCR_negative_BOD(SCR_negative_BODC[2])
);
counter_6bits OPTC4(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[3]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[3]),
	.o_SCR_negative_state(SCR_negative_stateC[3]),
	.o_SCR_forward_BOD(SCR_forward_BODC[3]),
	.o_SCR_negative_BOD(SCR_negative_BODC[3])
);
counter_6bits OPTC5(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[4]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[4]),
	.o_SCR_negative_state(SCR_negative_stateC[4]),
	.o_SCR_forward_BOD(SCR_forward_BODC[4]),
	.o_SCR_negative_BOD(SCR_negative_BODC[4])
);
counter_6bits OPTC6(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[5]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[5]),
	.o_SCR_negative_state(SCR_negative_stateC[5]),
	.o_SCR_forward_BOD(SCR_forward_BODC[5]),
	.o_SCR_negative_BOD(SCR_negative_BODC[5])
);
counter_6bits OPTC7(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[6]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[6]),
	.o_SCR_negative_state(SCR_negative_stateC[6]),
	.o_SCR_forward_BOD(SCR_forward_BODC[6]),
	.o_SCR_negative_BOD(SCR_negative_BODC[6])
);
counter_6bits OPTC8(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[7]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[7]),
	.o_SCR_negative_state(SCR_negative_stateC[7]),
	.o_SCR_forward_BOD(SCR_forward_BODC[7]),
	.o_SCR_negative_BOD(SCR_negative_BODC[7])
);
counter_6bits OPTC9(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[8]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[8]),
	.o_SCR_negative_state(SCR_negative_stateC[8]),
	.o_SCR_forward_BOD(SCR_forward_BODC[8]),
	.o_SCR_negative_BOD(SCR_negative_BODC[8])
);
counter_6bits OPTC10(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[9]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[9]),
	.o_SCR_negative_state(SCR_negative_stateC[9]),
	.o_SCR_forward_BOD(SCR_forward_BODC[9]),
	.o_SCR_negative_BOD(SCR_negative_BODC[9])
);
counter_6bits OPTC11(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[10]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[10]),
	.o_SCR_negative_state(SCR_negative_stateC[10]),
	.o_SCR_forward_BOD(SCR_forward_BODC[10]),
	.o_SCR_negative_BOD(SCR_negative_BODC[10])
);
counter_6bits OPTC12(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[11]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[11]),
	.o_SCR_negative_state(SCR_negative_stateC[11]),
	.o_SCR_forward_BOD(SCR_forward_BODC[11]),
	.o_SCR_negative_BOD(SCR_negative_BODC[11])
);
counter_6bits OPTC13(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[12]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[12]),
	.o_SCR_negative_state(SCR_negative_stateC[12]),
	.o_SCR_forward_BOD(SCR_forward_BODC[12]),
	.o_SCR_negative_BOD(SCR_negative_BODC[12])
);
counter_6bits OPTC14(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[13]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[13]),
	.o_SCR_negative_state(SCR_negative_stateC[13]),
	.o_SCR_forward_BOD(SCR_forward_BODC[13]),
	.o_SCR_negative_BOD(SCR_negative_BODC[13])
);
counter_6bits OPTC15(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[14]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[14]),
	.o_SCR_negative_state(SCR_negative_stateC[14]),
	.o_SCR_forward_BOD(SCR_forward_BODC[14]),
	.o_SCR_negative_BOD(SCR_negative_BODC[14])
);
counter_6bits OPTC16(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[15]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[15]),
	.o_SCR_negative_state(SCR_negative_stateC[15]),
	.o_SCR_forward_BOD(SCR_forward_BODC[15]),
	.o_SCR_negative_BOD(SCR_negative_BODC[15])
);
counter_6bits OPTC17(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[16]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[16]),
	.o_SCR_negative_state(SCR_negative_stateC[16]),
	.o_SCR_forward_BOD(SCR_forward_BODC[16]),
	.o_SCR_negative_BOD(SCR_negative_BODC[16])
);
counter_6bits OPTC18(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[17]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[17]),
	.o_SCR_negative_state(SCR_negative_stateC[17]),
	.o_SCR_forward_BOD(SCR_forward_BODC[17]),
	.o_SCR_negative_BOD(SCR_negative_BODC[17])
);
counter_6bits OPTC19(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[18]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[18]),
	.o_SCR_negative_state(SCR_negative_stateC[18]),
	.o_SCR_forward_BOD(SCR_forward_BODC[18]),
	.o_SCR_negative_BOD(SCR_negative_BODC[18])
);
counter_6bits OPTC20(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[19]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[19]),
	.o_SCR_negative_state(SCR_negative_stateC[19]),
	.o_SCR_forward_BOD(SCR_forward_BODC[19]),
	.o_SCR_negative_BOD(SCR_negative_BODC[19])
);
counter_6bits OPTC21(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[20]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[20]),
	.o_SCR_negative_state(SCR_negative_stateC[20]),
	.o_SCR_forward_BOD(SCR_forward_BODC[20]),
	.o_SCR_negative_BOD(SCR_negative_BODC[20])
);
counter_6bits OPTC22(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[21]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[21]),
	.o_SCR_negative_state(SCR_negative_stateC[21]),
	.o_SCR_forward_BOD(SCR_forward_BODC[21]),
	.o_SCR_negative_BOD(SCR_negative_BODC[21])
);
counter_6bits OPTC23(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[22]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[22]),
	.o_SCR_negative_state(SCR_negative_stateC[22]),
	.o_SCR_forward_BOD(SCR_forward_BODC[22]),
	.o_SCR_negative_BOD(SCR_negative_BODC[22])
);
counter_6bits OPTC24(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[23]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[23]),
	.o_SCR_negative_state(SCR_negative_stateC[23]),
	.o_SCR_forward_BOD(SCR_forward_BODC[23]),
	.o_SCR_negative_BOD(SCR_negative_BODC[23])
);
counter_6bits OPTC25(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[24]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[24]),
	.o_SCR_negative_state(SCR_negative_stateC[24]),
	.o_SCR_forward_BOD(SCR_forward_BODC[24]),
	.o_SCR_negative_BOD(SCR_negative_BODC[24])
);
counter_6bits OPTC26(
	.i_clk_50m(i_clk_50m),
	.i_signal(i_Opt_Receive_State_c[25]),
	.i_signal_forward(Opt_forward_pulse[2]),
	.i_signal_negative(Opt_negative_pulse[2]),
	.i_signal_forbid(pulse_forbid_state),
	.o_SCR_forward_state(SCR_forward_stateC[25]),
	.o_SCR_negative_state(SCR_negative_stateC[25]),
	.o_SCR_forward_BOD(SCR_forward_BODC[25]),
	.o_SCR_negative_BOD(SCR_negative_BODC[25])
);


//2. 数据更新至双口RAM
/*
reg [15:0] Fire_Enable=16'h5555;
reg [15:0] Breakdown_Trip=16'hAAAA;
reg [15:0] Work_State=16'hAAAA;

reg[22:0] counter_forward_pulse_60ms=23'd0;
reg[22:0] counter_negative_pulse_60ms=23'd0;

always @(posedge bus_clk_50m)
begin
	if(Fire_Enable==16'hAAAA)
	begin
		
		Opt_forward_pulse_buff<=Opt_forward_pulse;
		Opt_negative_pulse_buff<=Opt_negative_pulse;
		
		if((Opt_forward_pulse_buff==1'b0)&&(Opt_forward_pulse==1'b1))
		begin
			counter_forward_pulse_60ms<=23'd0;
		end
		 else
		 begin
			if(counter_forward_pulse_60ms>=23'd3000000)
				counter_forward_pulse_60ms<=23'd3000000;
			else
				counter_forward_pulse_60ms<=counter_forward_pulse_60ms+23'd1;
		end
		
		if((Opt_negative_pulse_buff==1'b0)&&(Opt_negative_pulse==1'b1))
		begin
			counter_negative_pulse_60ms<=23'd0;
		end
		 else
		 begin
			if(counter_negative_pulse_60ms>=23'd3000000)
				counter_negative_pulse_60ms<=23'd3000000;
			else
				counter_negative_pulse_60ms<=counter_negative_pulse_60ms+23'd1;
		end
	end
	else
	begin
		counter_forward_pulse_60ms<=23'd0;
		counter_negative_pulse_60ms<=23'd0;
	end
end

always @(posedge bus_clk_50m)
begin
	if((Fire_Enable==16'hAAAA)&&(counter_forward_pulse_60ms < 23'd3000000)&&(counter_negative_pulse_60ms < 23'd3000000))
		pulse_forbid_state<=1'b0;
	else
		pulse_forbid_state<=1'b1;
		
	if(Fire_Enable==16'hAAAA)
		optm_SCR_flag_state[1:0]<=2'b10;
	else
		optm_SCR_flag_state[1:0]<=2'b01;
		
	if(counter_forward_pulse_60ms >= 23'd3000000)
		optm_SCR_flag_state[3:2]<=2'b01;
	else
		optm_SCR_flag_state[3:2]<=2'b10;
		
	if(counter_negative_pulse_60ms >= 23'd3000000)
		optm_SCR_flag_state[5:4]<=2'b01;
	else
		optm_SCR_flag_state[5:4]<=2'b10;

	optm_SCR_forward_BOD<={3'b0, SCR_forward_BOD[12:0]};
	optm_SCR_negative_BOD<={3'b0, SCR_negative_BOD[12:0]};	
	
	optm_SCR_forward_bank<={3'b0, SCR_forward_bank[12:0]};	
	optm_SCR_negative_bank<={3'b0, SCR_negative_bank[12:0]};
	
	
	optm_SCR_forward_state<={3'b0, SCR_forward_state[12:0]};
	optm_SCR_negative_state<={3'b0, SCR_negative_state[12:0]};
	
	optm_SCR_forward_power<={3'b0, SCR_forward_power[12:0]};
	optm_SCR_negative_power<={3'b0, SCR_negative_power[12:0]};	
end*/

reg [25:0] optm_SCR_forward_stateA=25'd0;
reg [25:0] optm_SCR_negative_stateA=25'd0;
reg [25:0] optm_SCR_forward_BODA=25'd0;
reg [25:0] optm_SCR_negative_BODA=25'd0;

reg [25:0] optm_SCR_forward_stateB=25'd0;
reg [25:0] optm_SCR_negative_stateB=25'd0;
reg [25:0] optm_SCR_forward_BODB=25'd0;
reg [25:0] optm_SCR_negative_BODB=25'd0;

reg [25:0] optm_SCR_forward_stateC=25'd0;
reg [25:0] optm_SCR_negative_stateC=25'd0;
reg [25:0] optm_SCR_forward_BODC=25'd0;
reg [25:0] optm_SCR_negative_BODC=25'd0;
always @(posedge i_clk_50m)
begin	
	optm_SCR_forward_stateA <= SCR_forward_stateA;
	optm_SCR_negative_stateA <= SCR_negative_stateA;
	optm_SCR_forward_stateB <= SCR_forward_stateB;
	optm_SCR_negative_stateB <= SCR_negative_stateB;
	optm_SCR_forward_stateC <= SCR_forward_stateC;
	optm_SCR_negative_stateC <= SCR_negative_stateC;
	
	optm_SCR_forward_BODA <= SCR_forward_BODA;
	optm_SCR_negative_BODA <= SCR_negative_BODA;
	optm_SCR_forward_BODB <= SCR_forward_BODB;
	optm_SCR_negative_BODB <= SCR_negative_BODB;
	optm_SCR_forward_BODC <= SCR_forward_BODC;
	optm_SCR_negative_BODC <= SCR_negative_BODC;
end
//3. 双口RAM,开发板的DEMO
//--------------------------ram---------------------------//
//例化双口ram模块//
	wire [15:0]b_dataout;
	wire [15:0]a_dataout;
	my_ram    u2(
					.address_a(a_addr),
					.address_b(i_A),
					.data_a(a_datain),
					.data_b(io_DB),
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
//产生复位信号//
	reg [3:0]cnt_rst = 4'd0;
	
	always @ (posedge i_clk_25m)
		if(cnt_rst == 4'd10)
			begin
				cnt_rst <= 4'd10;
			end
		else
			cnt_rst <= cnt_rst + 4'd1;
			
	wire rst_n = (cnt_rst == 4'd10) ? 1'd1 : 1'd0;
	
//--------------------------delay---------------------------//
//对i_clk_25m时钟做延时处理//
	reg clk1,clk2;
	
	always @(posedge i_clk_100m or negedge rst_n) 
		begin
			if(!rst_n)
				begin 
					clk1 <= 1'd0;
					clk2 <= 1'd0;
				end 
			else
				{clk2,clk1} <= {clk1,i_clk_25m};
		end
		
	wire a_clk = (i_clk_25m & clk1);		
	
//--------------------------cnt----------------------------//
//ram地址数据计数器//
	reg [9:0]cnt; 
	
	always @ (posedge i_clk_25m or negedge rst_n)
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
//实现ram_a端口的写与读功能，计数器0到255为写，256到511为读//
  reg a_wren,a_rden;
  reg [15:0]a_datain;
  reg [9:0]a_addr;
//  reg [7:0]i;
  
	always @ (posedge a_clk or negedge rst_n)
		begin
			if(!rst_n)
			   begin
					a_wren <= 1'd0;
					a_rden <= 1'd0;
					a_datain <= 16'd0;
					a_addr <= 10'd0;
			   end 
			else if(cnt >= 10'd0 && cnt <=10'd31)
				begin
					a_wren <= 1'd1;                        //写使能信号拉高
					a_addr <= cnt;									//写地址
					//a_datain <= cnt;								//写入数据
					if(cnt==10'd0)
						a_datain <= optm_SCR_forward_stateA[15:0];
					else if(cnt==10'd1)
						a_datain <= {6'b0,optm_SCR_forward_stateA[25:16]};
					else if(cnt==10'd2)
						a_datain <= optm_SCR_negative_stateA[15:0];
					else if(cnt==10'd3)
						a_datain <= {6'b0,optm_SCR_negative_stateA[25:16]};	
					else if(cnt==10'd4)
						a_datain <= optm_SCR_forward_stateB[15:0];
					else if(cnt==10'd5)
						a_datain <= {6'b0,optm_SCR_forward_stateB[25:16]};
					else if(cnt==10'd6)
						a_datain <= optm_SCR_negative_stateB[15:0];
					else if(cnt==10'd7)
						a_datain <= {6'b0,optm_SCR_negative_stateB[25:16]};
					else if(cnt==10'd8)
						a_datain <= optm_SCR_forward_stateC[15:0];
					else if(cnt==10'd9)
						a_datain <= {6'b0,optm_SCR_forward_stateC[25:16]};
					else if(cnt==10'd10)
						a_datain <= optm_SCR_negative_stateC[15:0];
					else if(cnt==10'd11)
						a_datain <= {6'b0,optm_SCR_negative_stateC[25:16]};
					else if(cnt==10'd12)
						a_datain <= optm_SCR_forward_BODA[15:0];
					else if(cnt==10'd13)
						a_datain <= {6'b0,optm_SCR_forward_BODA[25:16]};
					else if(cnt==10'd14)
						a_datain <= optm_SCR_negative_BODA[15:0];
					else if(cnt==10'd15)
						a_datain <= {6'b0,optm_SCR_negative_BODA[25:16]};
					else if(cnt==10'd16)
						a_datain <= optm_SCR_forward_BODB[15:0];
					else if(cnt==10'd17)
						a_datain <= {6'b0,optm_SCR_forward_BODB[25:16]};
					else if(cnt==10'd18)
						a_datain <= optm_SCR_negative_BODB[15:0];
					else if(cnt==10'd19)
						a_datain <= {6'b0,optm_SCR_negative_BODB[25:16]};
					else if(cnt==10'd20)
						a_datain <= optm_SCR_forward_BODC[15:0];
					else if(cnt==10'd21)
						a_datain <= {6'b0,optm_SCR_forward_BODC[25:16]};
					else if(cnt==10'd22)
						a_datain <= optm_SCR_negative_BODC[15:0];
					else if(cnt==10'd23)
						a_datain <= {6'b0,optm_SCR_negative_BODC[25:16]};
					else if(cnt==10'd24)
						a_datain <= 16'h0;
					else if(cnt==10'd25)
						a_datain <= 16'h0;
					else if(cnt==10'd26)
						a_datain <= 16'h0;
					else if(cnt==10'd27)
						a_datain <= 16'h0;
					else if(cnt==10'd28)
						a_datain <= 16'h0;
					else if(cnt==10'd29)
						a_datain <= 16'h0;
					else if(cnt==10'd30)
						a_datain <= 16'h0;
					else if(cnt==10'd31)
						a_datain <= 16'h0;	
					a_rden <= 1'd0;								//读使能信号拉低
				end 
			else if(cnt >= 10'd32 && cnt <= 10'd63)
				begin
					a_rden <= 1'd1;								//读使能信号拉高
					a_addr <= cnt - 10'd32;					//读地址
					a_wren <= 1'd0;								//写使能信号拉低
				end
		end	
	
//--------------------------ram_a_error---------------------------//	
//错误判断标志，读取的地址和数据相等时标志为0，不相等时为1//
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
//根据错误标志控制led灯的颜色，无误时绿灯亮，有误时红灯亮//
//	reg ledr,ledg,ledb;		
	always @(posedge error or negedge rst_n) 
		if(!rst_n)
			begin
				o_Led_Error <= 1'd1;
				o_Led_Run <= 1'd0;
				o_Led_Other <= 1'd1;		
			end
		else 
			begin
				o_Led_Error <= 1'd0;
				o_Led_Run <= 1'd1;
				o_Led_Other <= 1'd1;
			end
		
	
//	assign {o_Led_Error,o_Led_Run,o_Led_Other} = {ledr,ledg,ledb};	
	
//--------------------------ram_b_rd---------------------------//
//控制ram_b端口的读取功能，当读信号来临时，读取相应的数据发送至fsmc总线//
	wire rd = (i_CS0 | i_RD);		 //提取读信号
	wire wr = (i_CS0 | i_WR);       //提取写信号
	
	reg wr_clk1,wr_clk2;
	always @(posedge i_clk_100m or negedge rst_n)
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
	assign io_DB = !rd ? b_dataout : 16'hzzzz;				//读信号来临，读取ram_b端口数据	
	
endmodule 