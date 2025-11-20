`include "define.svh"
`define MAX_CH 4

module PE (
    input clk,
    input rst,
    input PE_en,
    input [`CONFIG_SIZE-1:0] i_config,
    input [`DATA_BITS-1:0] ifmap,
    input [`DATA_BITS-1:0] filter,
    input [`DATA_BITS-1:0] ipsum,
    input ifmap_valid,
    input filter_valid,
    input ipsum_valid,
    input opsum_ready,
    output logic [`DATA_BITS-1:0] opsum,
    output logic ifmap_ready,
    output logic filter_ready,
    output logic ipsum_ready,
    output logic opsum_valid
);

typedef enum logic [2:0] {
    S_READ_CONFIG,
    S_READ_FILTER,
    S_READ_IFMAP1, // Read first S=3 ifmap
    S_READ_IFMAP2, // Read 1 ifmap and pop 1 ifmap
    S_READ_IPSUM,  // Read p ipsum
    S_ACCUMULATE,  // Accumulate psum += filter * ifmap
    S_WRITE_OPSUM, // Write p opsum to GLB
    S_DONE
} state_t;

// state reg
state_t state, next_state;

// spad
logic signed [`IFMAP_SIZE-1:0]  ifmap_spad  [`IFMAP_SPAD_LEN-1:0];
logic signed [`FILTER_SIZE-1:0] filter_spad [`FILTER_SPAD_LEN-1:0];
logic signed [`PSUM_SIZE-1:0]   ofmap_spad  [`OFMAP_SPAD_LEN-1:0];

// control signals
logic [5:0] counter;
logic [1:0] m_idx; // filter index
logic [1:0] c_idx; // channel index
logic [1:0] f_idx; // kernel index
logic [4:0] f_count; // number of 1D convolution

// config
logic [1:0] q; // q - 1
logic [4:0] F; // F
logic [1:0] p; // p - 1
logic mode;

always_comb begin
    case (state)
        S_READ_CONFIG:
            next_state = PE_en ? S_READ_FILTER : S_READ_CONFIG;

        S_READ_FILTER:
            next_state = ((counter == `FILTER_SPAD_LEN - `MAX_CH) && filter_valid) ?
                         S_READ_IFMAP1 : S_READ_FILTER;

        S_READ_IFMAP1:
            next_state = ((counter == `IFMAP_SPAD_LEN - `MAX_CH) && ifmap_valid) ?
                         S_READ_IPSUM : S_READ_IFMAP1;

        S_READ_IFMAP2:
            next_state = ifmap_valid ? S_READ_IPSUM : S_READ_IFMAP2;

        S_READ_IPSUM:
            next_state = (2'(counter) == p && ipsum_valid) ?
                         S_ACCUMULATE : S_READ_IPSUM;

        S_ACCUMULATE:
            next_state = (c_idx == q && m_idx == p && f_idx == `FILT_S - 1) ?
                         S_WRITE_OPSUM : S_ACCUMULATE;

        S_WRITE_OPSUM:
            next_state = (2'(counter) == p && opsum_ready) ?
                         (f_count == F + 1 ? S_DONE : S_READ_IFMAP2) :
                         S_WRITE_OPSUM;

        S_DONE:
            next_state = S_DONE;

        default:
            next_state = S_READ_CONFIG;
    endcase
end

// ===================================================================
//  Data
// ===================================================================

// config p, q
always_ff @(posedge clk) begin
    case (state)
        S_READ_CONFIG: begin
            q    <= i_config[1:0];
            F    <= i_config[6:2];
            p    <= i_config[8:7];
            mode <= i_config[9];
        end
        default: begin
            q    <= q;
            F    <= F;
            p    <= p;
            mode <= mode;
        end
    endcase
end

// spad
always_ff @(posedge clk) begin
    case (state)
        S_READ_FILTER: begin
            for (int i = 0; i < 4; i = i + 1) begin
                filter_spad[counter + 6'(i)] <= filter[i*`FILTER_SIZE +: `FILTER_SIZE];
            end
        end

        S_READ_IFMAP1: begin
            for (int i = 0; i < 4; i = i + 1) begin
                /* ifmap - 128 = {ifmap[7] ^ 1'b1, ifmap[6:0]}, if ifmap is int8 */
                ifmap_spad[4'(counter) + 4'(i)] <= {~ifmap[(i+1)*`IFMAP_SIZE - 1],
                                                    ifmap[i*`IFMAP_SIZE +: `IFMAP_SIZE-1]};
            end
        end

        S_READ_IFMAP2: begin
            if (ifmap_valid)
                for (int i = 0; i < 4; i = i + 1) begin
                    ifmap_spad[i] <= ifmap_spad[4 + i];
                    ifmap_spad[4 + i] <= ifmap_spad[8 + i];
                    ifmap_spad[8 + i] <= {~ifmap[(i+1)*`IFMAP_SIZE - 1],
                                          ifmap[i*`IFMAP_SIZE +: `IFMAP_SIZE-1]};
                end
        end

        S_READ_IPSUM: begin
            ofmap_spad[counter[1:0]] <= ipsum;
        end

        S_ACCUMULATE: begin
            ofmap_spad[m_idx] <= ofmap_spad[m_idx] +
                                 (filter_spad[6'(m_idx) * 12 + (6'(f_idx) << 2) + 6'(c_idx)] *
                                  ifmap_spad[(4'(f_idx) << 2) | 4'(c_idx)]);
        end

        default:
            filter_spad[counter] <= filter_spad[counter];
    endcase
end

// ===================================================================
//  Control
// ===================================================================

// counter
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        counter <= 0;
    else
    case (state)
        S_READ_FILTER:
            if (filter_valid)
                if (counter == `FILTER_SPAD_LEN - `MAX_CH)
                    counter <= 0;
                else
                    counter <= counter + `MAX_CH;
            else
                counter <= counter;

        S_READ_IFMAP1:
            if (ifmap_valid)
                if (counter == `IFMAP_SPAD_LEN - `MAX_CH)
                    counter <= 0;
                else
                    counter <= counter + `MAX_CH;
            else
                counter <= counter;

        S_READ_IPSUM:
            if (ipsum_valid)
                counter <= (2'(counter) == p) ? 0 : counter + 1;
            else
                counter <= counter;

        S_WRITE_OPSUM:
            if (opsum_ready)
                counter <= (2'(counter) == p) ? 0 : counter + 1;
            else
                counter <= counter;

        default:
            counter <= 0;
    endcase
end

// c_idx
always_ff @(posedge clk) begin
    case (state)
        S_ACCUMULATE:
            c_idx <= (m_idx == p && f_idx == `FILT_S - 1) ? c_idx + 1 : c_idx;
        default:
            c_idx <= 0;
    endcase
end

// m_idx
always_ff @(posedge clk) begin
    case (state)
        S_ACCUMULATE:
            m_idx <= (f_idx == `FILT_S - 1) ? (m_idx == p ? 0 : m_idx + 1) : m_idx;
        default:
            m_idx <= 0;
    endcase
end

// f_idx
always_ff @(posedge clk) begin
    case (state)
        S_ACCUMULATE:
            f_idx <= (f_idx == `FILT_S - 1) ? 0 : f_idx + 1;
        default:
            f_idx <= 0;
    endcase
end

// f_count
always_ff @(posedge opsum_valid or posedge rst) begin
    if (rst)
        f_count <= 0;
    else
        f_count <= f_count + 1;
end

// state register
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        state <= S_READ_CONFIG;
    else
        state <= next_state;
end

// ===================================================================
//  Output
// ===================================================================

always_comb begin
    filter_ready = (state == S_READ_FILTER);
    ifmap_ready  = (state == S_READ_IFMAP1 || state == S_READ_IFMAP2);
    ipsum_ready  = (state == S_READ_IPSUM);
    opsum_valid  = (state == S_WRITE_OPSUM);
    opsum        = ofmap_spad[2'(counter)];
end

endmodule
