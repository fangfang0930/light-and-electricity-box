//
//
//
//
//

module main_start(
  input wire i_clk_50M, //50MHz时钟输入
  input wire dip3,  //拨码3输入状态
  input wire dip4,  //拨码4输入状态
  output reg o_start    //启动主程序标志
);
reg[31:0] cnt = 32'd0;  //累加计数
reg[1:0] dip;
reg end_flag = 1'b0;
//延时 0x00:15s,0x01:30s,0x10:45s,0x11:60s，最大,60s
always@(posedge i_clk_50M) begin
  dip <= {~dip4,~dip3};
  cnt <= cnt + 1'd1;
  case(dip)
    2'd0: begin  //15s
      if((cnt < 32'd750000000)&&(!end_flag)) begin
        o_start <= 1'b0;   
      end
      else begin
        cnt <= 32'd750000002;
	     o_start <= 1'b1;
		  end_flag <= 1'b1;
      end   
    end
    2'd1: begin  //30s
      if((cnt < 32'd1500000000)&&(!end_flag))  begin
        o_start <= 1'b0;   
      end
      else begin
        cnt <= 32'd1500000002;
	     o_start <= 1'b1;
		  end_flag <= 1'b1;
      end   
    end
    2'd2: begin  //45s
      if((cnt < 32'd2250000000)&&(!end_flag))  begin
        o_start <= 1'b0;   
      end
      else begin
        cnt <= 32'd2250000002;
	     o_start <= 1'b1;
		  end_flag <= 1'b1;
      end   
    end
    2'd3: begin  //60s
      if((cnt < 32'd3000000000)&&(!end_flag))  begin
        o_start <= 1'b0;   
      end
      else begin
        cnt <= 32'd3000000002;
	     o_start <= 1'b1;
		  end_flag <= 1'b1;
      end  
    end
    default: begin  //默认60s
      if((cnt < 32'd3000000000)&&(!end_flag))  begin
        o_start <= 1'b0;   
      end
      else begin
        cnt <= 32'd3000000002;
	     o_start <= 1'b1;
		  end_flag <= 1'b1;
      end
    end		
  endcase  
end
endmodule
