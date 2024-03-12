`timescale 1ns/1ns

module counter_6bits_tb;

    // Parameters
    parameter CLK_PERIOD = 2; // 时钟周期，单位：ns

    // Signals
    reg i_clk_50m;
    reg i_signal, i_signal_forward, i_signal_negative, i_signal_forbid;
    wire o_SCR_forward_state, o_SCR_negative_state, o_SCR_forward_BOD, o_SCR_negative_BOD;

    // Instantiate the Unit Under Test (UUT)
    counter_6bits uut (
        .i_clk_50m(i_clk_50m),
        .i_signal(i_signal),
        .i_signal_forward(i_signal_forward),
        .i_signal_negative(i_signal_negative),
        .i_signal_forbid(i_signal_forbid),
        .o_SCR_forward_state(o_SCR_forward_state),
        .o_SCR_negative_state(o_SCR_negative_state),
        .o_SCR_forward_BOD(o_SCR_forward_BOD),
        .o_SCR_negative_BOD(o_SCR_negative_BOD)
    );

    // Clock Generation
   // always begin
    //    #CLK_PERIOD  i_clk_50m = ~i_clk_50m; // 50MHz的时钟信号，每个周期切换一次
   // end
    // Clock Generation
    initial begin
        i_clk_50m = 0;
        forever #2  i_clk_50m = ~i_clk_50m; // 50MHz的时钟信号，每个周期切换一次
    end
    // Stimulus
    initial begin
        // 初始化输入信号
        i_signal = 0;
        i_signal_forward = 0;
        i_signal_negative = 0;
        i_signal_forbid = 0;

        // 等待50个时钟周期
        #5;

        // 测试第一种情况
        // 设置击穿信号为1，持续几个时钟周期
        i_signal = 1;
        #500;
        i_signal = 0;

        // 等待一段时间
        #20000;

        // 测试第二种情况
        // 设置正相触发脉冲为1
        i_signal_forward = 1;
        #10000;
        i_signal_forward = 0;

        // 等待一段时间
      //  #25;

        // 测试第三种情况
        // 设置脉冲禁止信号为1
        i_signal_forbid = 0;
       // #10;
       // i_signal_forbid = 0;

        // 等待一段时间
        #24000;
		
        i_signal = 1;
        #10000;
        i_signal = 0;

        // 结束仿真
      //  $stop;
    end

endmodule
