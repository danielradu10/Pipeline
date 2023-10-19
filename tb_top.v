

module tb_top(

    );
    reg clk;
    reg res;
    
    top DUT(
        .clk(clk),
        .res(res)
    );
    
    initial begin
        clk = 0; res = 0;
        #10 res = 1;
        #10 res = 0;
        #200 $finish;
    end

    always #5 clk = ~clk;
endmodule
