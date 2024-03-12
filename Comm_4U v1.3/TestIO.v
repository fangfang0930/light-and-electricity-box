//COMM板卡的IO测试
//
//
//

//模块声明
module TeseIO(
input wire i_clk_50M,
input wire i_start,//启动输入
input wire i_rx1,//COMM-RX1 from Optic
input wire i_rx2,//COMM-RX2 from Optic
input wire i_rx3,//COMM-RX3 from Optic
input wire i_di0,//Feedback Pulse_AB+ from Optical Receiving board1
input wire i_di1,//Feedback Pulse_AB- from Optical Receiving board1
input wire i_di2,//Feedback Pulse_BC+ from Optical Receiving board1
input wire i_di3,//Feedback Pulse_BC- from Optical Receiving board1
input wire i_di4,//Feedback Pulse_CA+ from Optical Receiving board1
input wire i_di5,//Feedback Pulse_CA- from Optical Receiving board1
input wire i_di6,//Dead from Optical Receiving board2
input wire i_di7,//Dead from Optical Receiving board2
input wire i_di8,//Dead from Optical Receiving board2
input wire i_di9,//BD_Fault from Optical Receiving board2
input wire i_di10,//BD_Fault from Optical Receiving board2
input wire i_di11,//BD_Fault from Optical Receiving board2
input wire i_di12,//Reserved from Optical Receiving board3
input wire i_di13,//Reserved from Optical Receiving board3
input wire i_di14,//Reserved from Optical Receiving board3
input wire i_di15,//Reserved from Optical Receiving board3
input wire i_di16,//Reserved from Optical Receiving board3
input wire i_di17,//Reserved from Optical Receiving board3
input wire i_di18,//COMM-TX1 from NI_DIO0
input wire i_di19,//COMM-TX2 from NI_DIO1
input wire i_di20,//COMM-TX3 from NI_DIO2
input wire i_di21,//Fire Pulse_AB+ from NI_DIO3
input wire i_di22,//Fire Pulse_AB- from NI_DIO4
input wire i_di23,//Fire Pulse_BC+ from NI_DIO5
input wire i_di24,//Fire Pulse_BC- from NI_DIO6
input wire i_di25,//Fire Pulse_CA+ from NI_DIO7
input wire i_di26,//Fire Pulse_CA- from NI_DIO8
input wire i_di27,//Dead from NI_DIO9
input wire i_di28,//Dead from NI_DI10
input wire i_di29,//Dead from NI_DI11
input wire i_di30,//Syn from NI_DI12
input wire i_di31,//Syn from NI_DI13
input wire i_di32,//Syn from NI_DI14
input wire i_di33,//Reserved from NI_DI15
output reg o_led_run,
output reg o_led_error,
output reg o_tx1,//COMM-TX1 to Optic
output reg o_tx2,//COMM-TX2 to Optic
output reg o_tx3,//COMM-TX3 to Optic
output reg o_do0,//Fire Pulse_AB+ to Optical Transmit board1
output reg o_do1,//Fire Pulse_AB- to Optical Transmit board1
output reg o_do2,//Fire Pulse_BC+ to Optical Transmit board1
output reg o_do3,//Fire Pulse_BC- to Optical Transmit board1
output reg o_do4,//Fire Pulse_CA+ to Optical Transmit board1
output reg o_do5,//Fire Pulse_CA- to Optical Transmit board1
output reg o_do6,//dead to Optical Transmit board2
output reg o_do7,//dead to Optical Transmit board2
output reg o_do8,//dead to Optical Transmit board2
output reg o_do9,//syn to Optical Transmit board2
output reg o_do10,//syn to Optical Transmit board2
output reg o_do11,//syn to Optical Transmit board2
output reg o_do12,//Reserved  to Optical Transmit board3
output reg o_do13,//Reserved  to Optical Transmit board3
output reg o_do14,//Reserved  to Optical Transmit board3
output reg o_do15,//Reserved  to Optical Transmit board3
output reg o_do16,//Reserved  to Optical Transmit board3	
output reg o_do17,//Reserved  to Optical Transmit board3
output reg o_do18,//COMM-RX1 to NI_DIO16
output reg o_do19,//COMM-RX2 to NI_DIO17
output reg o_do20,//COMM-RX3 to NI_DIO18
output reg o_do21,//Feedback Pulse_AB+ to NI_DIO19
output reg o_do22,//Feedback Pulse_AB- to NI_DIO20
output reg o_do23,//Feedback Pulse_BC+ to NI_DIO21
output reg o_do24,//Feedback Pulse_BC- to NI_DIO22
output reg o_do25,//Feedback Pulse_CA+ to NI_DIO23
output reg o_do26,//Feedback Pulse_CA- to NI_DIO24
output reg o_do27,//Feeback Dead to NI_DIO25
output reg o_do28,//Feeback Dead to NI_DIO26
output reg o_do29,//Feeback Dead to NI_DIO27
output reg o_do30,//BD_Fault to NI_DIO28
output reg o_do31,//BD_Fault to NI_DIO29
output reg o_do32,//BD_Fault to NI_DIO30
output reg o_do33 //Reserved to NI_DIO31
);

//变量定义
reg[23:0] c_led_run = 24'd0; //运行指示灯闪烁
parameter   COUNT_3o3ms=24'd165000; //3.3ms
parameter   COUNT_2o8ms=24'd140000; //2.8ms

parameter   COUNT_6o6ms=24'd330000; //6.1ms

parameter   COUNT_6o1ms=24'd305000; //6.1ms

parameter   COUNT_0o5ms=24'd25000; //0.5ms

parameter	check_time1=24'd25000;//0.5
parameter	check_time2=24'd330000; //6.6ms
parameter	check_time3=24'd355000;//330000+25000
parameter	check_time4=24'd660000;//355000+305000
parameter	check_time5=24'd685000;//
//parameter	check_time6=24'd990000;//
//parameter	check_time7=24'd1,015,000;
//parameter	check_time8=24'd1,320,000;
//parameter	check_time9=24'd1,345,000;



  parameter CLOCK_FREQUENCY = 50000000; // Hz
  parameter PULSE_WIDTH = 500000; // ns (0.5ms)
  parameter CYCLE_DURATION = 6600000; // ns (6.6ms)
  parameter COUNTER_MAX =COUNT_3o3ms;

  reg [23:0] counter;
  reg [5:0] state;
  reg [5:0] o_do;//<={o_do5,o_do4,o_do3,o_do2,o_do1,o_do0};
  initial
  begin
      counter <= 0;
      state <= 0;
	  o_do <= 6'b111111;//0;
  end
  always @(posedge i_clk_50M ) begin
  // if (!i_start) begin
  //    counter <= 0;
  //    state <= 0;
   // end else 
   begin
      counter <= counter + 1;
      if (counter == COUNTER_MAX+1) begin
        counter <= 0;
        state <= state + 1;
      end
    end
	o_do<={o_do5,o_do4,o_do3,o_do2,o_do1,o_do0};
 // end

////  always @(posedge i_clk_50M) begin
   // if (!i_start) begin
   //   o_do <= 0;
    //end else 
	begin
      case (state)
        0: begin
		o_do0<=0;
		o_do1<=0;
		o_do2<=0;
		o_do3<=0;
		o_do4<=0;
		o_do5<=0;
          //o_do <= 6'b111111; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        1: begin
         // o_do <= 6'b001000; // 0.5ms high pulse
		 	o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=1;
			o_do4<=0;
			o_do5<=0;
          if (counter == COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        2: begin
		 	o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
          //o_do <= 6'b000000; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        3: begin
			o_do0<=1;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
         // o_do <= 6'b000001; // 0.5ms high pulse
          if (counter == COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        4: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
         // o_do <= 6'b000000; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        5: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=1;
          o_do <= 6'b100000; // 0.5ms high pulse
          if (counter == COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        6: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
          //o_do <= 6'b000000; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        7: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=1;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
        //  o_do <= 6'b000100; // 0.5ms high pulse
          if (counter == COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        8: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
         // o_do <= 6'b000000; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        9: begin
			o_do0<=0;
			o_do1<=1;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
          //o_do <= 6'b000010; // 0.5ms high pulse
          if (counter == COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        10: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=0;
			o_do5<=0;
         // o_do <= 6'b000000; // 6.1ms low pulse
          if (counter == COUNTER_MAX-COUNT_0o5ms) begin
            counter <= 0;
            state <= state + 1;
          end
        end
        11: begin
			o_do0<=0;
			o_do1<=0;
			o_do2<=0;
			o_do3<=0;
			o_do4<=1;
			o_do5<=0;
          //o_do <= 6'b010000; // 0.5ms high pulse
          if (counter == COUNT_0o5ms) begin
           counter <= 0;
           state <= 0;// state + 1;
		  end
	    end
	   default: o_do <= 6'b000000;
	endcase
	end
end

always@(posedge i_clk_50M) begin
  if(i_start) begin
    //通讯板3路发射光头TX1~TX3
    o_tx1 <= i_di18;
    o_tx2 <= i_di19;
    o_tx3 <= i_di20;
	 //光发射板1光头1~6，6路触发脉冲,500us
 //  o_do0 <= i_di21;
 //  o_do1 <= i_di22;
 //  o_do2 <= i_di23;
 //  o_do3 <= i_di24;
 //  o_do4 <= i_di25;
 //  o_do5 <= i_di26;
	 //光发射板2光头1~6，6路触发脉冲,500us，Only for test 2023.04.22
    o_do6 <= i_di21;
    o_do7 <= i_di22;
    o_do8 <= i_di23;
    o_do9 <= i_di24;
    o_do10 <= i_di25;
    o_do11 <= i_di26;
	 //光发射板3光头1，预留
    o_do12 <= i_di33;	 
	 
	 //通讯板3路接收光头RX1~RX3
    o_do18 <= i_rx1;
    o_do19 <= i_rx2;
    o_do20 <= i_rx3;
	 //光接收板1光头1~6，6路触发脉冲回馈,接收光电5-5-5信号扩展为500us脉冲，用于主控检丢脉冲 2023.04.22
    /*o_do21 <= i_di0;
    o_do22 <= i_di1;
    o_do23 <= i_di2;
    o_do24 <= i_di3;
    o_do25 <= i_di4;
    o_do26 <= i_di5;*/

	 
	 //光接收板2光头1~6，3路死机，3路预留
    o_do27 <= i_di6;
    o_do28 <= i_di7;
    o_do29 <= i_di8;
    o_do30 <= i_di9;
    o_do31 <= i_di10;
    o_do32 <= i_di11;
	 //光接收板3光头1
    o_do33 <= i_di12;
	 
	 //LED操作，闪烁间隔100ms
	 o_led_error <= 1'b1; //灭故障LED
	 c_led_run <= c_led_run + 1'd1;
    if(c_led_run >= 24'd4999999) begin
      c_led_run <= 24'd0;
	   o_led_run <= ~o_led_run;   //闪烁运行LED
    end
  end
  else begin
    //通讯板3路发射光头TX1~TX3
    o_tx1 <= 1'b0;
    o_tx2 <= 1'b0;
    o_tx3 <= 1'b0;
	 //光发射板1光头1~6
  //  o_do0 <= 1'b0;
  //  o_do1 <= 1'b0;
  //  o_do2 <= 1'b0;
  //  o_do3 <= 1'b0;
  //  o_do4 <= 1'b0;
  //  o_do5 <= 1'b0;
    //光发射板2光头1~6
	 o_do6 <= 1'b0;
	 o_do7 <= 1'b0;
	 o_do8 <= 1'b0;
	 o_do9 <= 1'b0;
	 o_do10 <= 1'b0;
	 o_do11 <= 1'b0;
	 //光发射板3光头1~6
	 o_do12 <= 1'b0;
	 o_do13 <= 1'b0;
	 o_do14 <= 1'b0;
	 o_do15 <= 1'b0;
	 o_do16 <= 1'b0;
	 o_do17 <= 1'b0;
	 
	 //通讯板3路接收光头RX1~RX3
    o_do18 <= 1'b0;
    o_do19 <= 1'b0;
    o_do20 <= 1'b0;
	 //光接收板1光头1~6
/*    o_do21 <= 1'b0;
    o_do22 <= 1'b0;
    o_do23 <= 1'b0;	 
    o_do24 <= 1'b0;
    o_do25 <= 1'b0;
    o_do26 <= 1'b0;*/
	 //光接收板2光头1~6
    o_do27 <= 1'b0;
    o_do28 <= 1'b0;
    o_do29 <= 1'b0;	 
    o_do30 <= 1'b0;
    o_do31 <= 1'b0;
    o_do32 <= 1'b0;	
	 //光接收板3光头1
    o_do33 <= 1'b0; 
    o_led_error <= 1'b0; //亮故障LED
    o_led_run <= 1'b1;   //灭运行LED	 
  end
end

reg[5:0] Opt_pulse_buff = 1'b0;
reg[5:0]  pulse_flag = 1'b0;
reg[5:0]  pulse_flag1 = 1'b0;
reg[7:0] counter_555[5:0];
reg[19:0] counter_555_flag[5:0];
reg[15:0] counter_555_flag1[5:0];

always@(posedge i_clk_50M) begin
//AB+回报500us
	 if((Opt_pulse_buff[0] == 1'b0)&&(i_di0 == 1'b1)) begin
		pulse_flag[0]<= 1'b1;
		counter_555[0] <= counter_555[0] + 8'd1;
    end		
	 if(pulse_flag[0]) begin
	   counter_555_flag1[0] <= counter_555_flag1[0] + 16'd1;
		if(counter_555_flag1[0] >= 16'd2500) begin
			counter_555_flag1[0] <= 16'd0;
			pulse_flag[0]<= 1'b0;
			if(counter_555[0] >= 2) 
				pulse_flag1[0] <= 1'b1;
			counter_555[0] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[0] <= i_di0;		
		
	 if(pulse_flag1[0] == 1'b1)
		begin
		 counter_555_flag[0] <= counter_555_flag[0] + 17'd1;
		 if(counter_555_flag[0] <= 17'd25000) o_do21 <= 1'b1;
		 else begin
			 counter_555_flag[0] <= 17'd0;
			 pulse_flag1[0] <= 1'b0;
			 o_do21 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[0] <= 17'd0;
			 o_do21 <= 1'b0;
		 end
//AB-回报500us
	 if((Opt_pulse_buff[1] == 1'b0)&&(i_di1 == 1'b1)) begin
		pulse_flag[1]<= 1'b1;
		counter_555[1] <= counter_555[1] + 8'd1;
    end		
	 if(pulse_flag[1]) begin
	   counter_555_flag1[1] <= counter_555_flag1[1] + 16'd1;
		if(counter_555_flag1[1] >= 16'd2500) begin
			counter_555_flag1[1] <= 16'd0;
			pulse_flag[1]<= 1'b0;
			if(counter_555[1] >= 2) 
				pulse_flag1[1] <= 1'b1;
			counter_555[1] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[1] <= i_di1;		
		
	 if(pulse_flag1[1] == 1'b1)
		begin
		 counter_555_flag[1] <= counter_555_flag[1] + 17'd1;
		 if(counter_555_flag[1] <= 17'd25000) o_do22 <= 1'b1;
		 else begin
			 counter_555_flag[1] <= 17'd0;
			 pulse_flag1[1] <= 1'b0;
			 o_do22 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[1] <= 17'd0;
			 o_do22 <= 1'b0;
		 end
//BC+回报500us
	 if((Opt_pulse_buff[2] == 1'b0)&&(i_di2 == 1'b1)) begin
		pulse_flag[2]<= 1'b1;
		counter_555[2] <= counter_555[2] + 8'd1;
    end		
	 if(pulse_flag[2]) begin
	   counter_555_flag1[2] <= counter_555_flag1[2] + 16'd1;
		if(counter_555_flag1[2] >= 16'd2500) begin
			counter_555_flag1[2] <= 16'd0;
			pulse_flag[2]<= 1'b0;
			if(counter_555[2] >= 2) 
				pulse_flag1[2] <= 1'b1;
			counter_555[2] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[2] <= i_di2;		
		
	 if(pulse_flag1[2] == 1'b1)
		begin
		 counter_555_flag[2] <= counter_555_flag[2] + 17'd1;
		 if(counter_555_flag[2] <= 17'd25000) o_do23 <= 1'b1;
		 else begin
			 counter_555_flag[2] <= 17'd0;
			 pulse_flag1[2] <= 1'b0;
			 o_do23 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[2] <= 17'd0;
			 o_do23 <= 1'b0;
		 end
//BC-回报500us
	 if((Opt_pulse_buff[3] == 1'b0)&&(i_di3 == 1'b1)) begin
		pulse_flag[3]<= 1'b1;
		counter_555[3] <= counter_555[3] + 8'd1;
    end		
	 if(pulse_flag[3]) begin
	   counter_555_flag1[3] <= counter_555_flag1[3] + 16'd1;
		if(counter_555_flag1[3] >= 16'd2500) begin
			counter_555_flag1[3] <= 16'd0;
			pulse_flag[3]<= 1'b0;
			if(counter_555[3] >= 2) 
				pulse_flag1[3] <= 1'b1;
			counter_555[3] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[3] <= i_di3;		
		
	 if(pulse_flag1[3] == 1'b1)
		begin
		 counter_555_flag[3] <= counter_555_flag[3] + 17'd1;
		 if(counter_555_flag[3] <= 17'd25000) o_do24 <= 1'b1;
		 else begin
			 counter_555_flag[3] <= 17'd0;
			 pulse_flag1[3] <= 1'b0;
			 o_do24 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[3] <= 17'd0;
			 o_do24 <= 1'b0;
		 end
//CA+回报500us
	 if((Opt_pulse_buff[4] == 1'b0)&&(i_di4 == 1'b1)) begin
		pulse_flag[4]<= 1'b1;
		counter_555[4] <= counter_555[4] + 8'd1;
    end		
	 if(pulse_flag[4]) begin
	   counter_555_flag1[4] <= counter_555_flag1[4] + 16'd1;
		if(counter_555_flag1[4] >= 16'd2500) begin
			counter_555_flag1[4] <= 16'd0;
			pulse_flag[4]<= 1'b0;
			if(counter_555[4] >= 2) 
				pulse_flag1[4] <= 1'b1;
			counter_555[4] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[4] <= i_di4;		
		
	 if(pulse_flag1[4] == 1'b1)
		begin
		 counter_555_flag[4] <= counter_555_flag[4] + 17'd1;
		 if(counter_555_flag[4] <= 17'd25000) o_do25 <= 1'b1;
		 else begin
			 counter_555_flag[4] <= 17'd0;
			 pulse_flag1[4] <= 1'b0;
			 o_do25 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[4] <= 17'd0;
			 o_do25 <= 1'b0;
		 end
//CA-回报500us
	 if((Opt_pulse_buff[5] == 1'b0)&&(i_di5 == 1'b1)) begin
		pulse_flag[5]<= 1'b1;
		counter_555[5] <= counter_555[5] + 8'd1;
    end		
	 if(pulse_flag[5]) begin
	   counter_555_flag1[5] <= counter_555_flag1[5] + 16'd1;
		if(counter_555_flag1[5] >= 16'd2500) begin
			counter_555_flag1[5] <= 16'd0;
			pulse_flag[5]<= 1'b0;
			if(counter_555[5] >= 2) 
				pulse_flag1[5] <= 1'b1;
			counter_555[5] <= 8'd0;
		end	 
	 end		
	 Opt_pulse_buff[5] <= i_di5;		
		
	 if(pulse_flag1[5] == 1'b1)
		begin
		 counter_555_flag[5] <= counter_555_flag[5] + 17'd1;
		 if(counter_555_flag[5] <= 17'd25000) o_do26 <= 1'b1;
		 else begin
			 counter_555_flag[5] <= 17'd0;
			 pulse_flag1[5] <= 1'b0;
			 o_do26 <= 1'b0;
			end
		end
	 else
		 begin
			 counter_555_flag[5] <= 17'd0;
			 o_do26 <= 1'b0;
		 end

end

endmodule
