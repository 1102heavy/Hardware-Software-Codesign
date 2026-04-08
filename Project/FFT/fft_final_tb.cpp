#include <iostream>
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_fixed.h>
#include <complex>
#include <cmath>

// Re-define types so the testbench understands the IP interface
typedef ap_fixed<16, 1, AP_RND_CONV, AP_SAT> data_t;
typedef std::complex<data_t> complex_t;
typedef ap_axis<32, 0, 0, 0> axis_pkt;
#define N 1024

// Function Prototype: This tells the testbench the IP exists in another file
void my_fft_only_dma_ip(
    hls::stream<axis_pkt>& data_in,
    hls::stream<axis_pkt>& data_out
);

int main() {
    // hls::stream<axis_pkt> config_in;
    hls::stream<axis_pkt> dma_to_ip;
    hls::stream<axis_pkt> ip_to_dma;
    float input_samples[N] = {0.0f};

    float fs = 44100.0;
    float freq = 10000.0; // We expect a peak at Bin ~23

    std::cout << "--- Starting C-Simulation ---" << std::endl;

    // 1. Pack Sine Wave into AXI-Stream (Real part only)
    // 1. Generate Input: A 1kHz Sine Wave at 50% Volume
    float amplitude = 0.5; // Represents 0.5 in Q1.15
    for (int i = 0; i < N; i++) {
        axis_pkt pkt;
        
        // Calculate the sine as a float (-1.0 to 1.0)
        float sample_f = amplitude * sin(2.0 * M_PI * freq * i / fs);
        input_samples[i]=sample_f;
        // Convert to the raw bits that represent Q1.15
        // 32768.0 is used to map 1.0 to the boundary of 16-bit signed
        int16_t real_val = (int16_t)(sample_f * 32767.0);
        
        pkt.data.range(15, 0)  = real_val;
        pkt.data.range(31, 16) = 0; // Imaginary is 0
        
        pkt.keep = -1; 
        pkt.last = (i == N - 1) ? 1 : 0;
        dma_to_ip.write(pkt);
    }

    // 2. Execute the IP
    // Note: config_in is passed but not used by your current logic
    my_fft_only_dma_ip(dma_to_ip, ip_to_dma);
    float recovered_samples[N] = {0.0f};
// // 3. Collect Recovered Samples fft-ifft
//     for (int i = 0; i < N; i++) {
//         if (!ip_to_dma.empty()) {
//             axis_pkt out_pkt = ip_to_dma.read();
//             int16_t raw_r = (int16_t)out_pkt.data.range(15, 0);
            
//             // Convert Q1.15 bits back to float -1.0 to 1.0
//             // No 1024 multiplier because you removed the IFFT shift!
//             recovered_samples[i] = (float)raw_r / 32768.0;
//         }
//     }

//     // 4. Calculate Statistics
//     float total_abs_error = 0.0;
//     float max_error = 0.0;

//     std::cout << "\n--- Sample Comparison (First 5) ---" << std::endl;
//     for (int i = 0; i < N; i++) {
//         float diff = std::abs(input_samples[i] - recovered_samples[i]);
//         total_abs_error += diff;
        
//         if (diff > max_error) max_error = diff;

//         if (i < 5) {
//             std::cout << "In: " << std::setw(8) << input_samples[i] 
//                     << " | Out: " << std::setw(8) << recovered_samples[i] 
//                     << " | Err: " << diff << std::endl;
//         }
//     }

//     float mae = total_abs_error / N;
//     std::cout << "---------------------------------" << std::endl;
//     std::cout << "Mean Absolute Error: " << mae << std::endl;
//     std::cout << "Max Deviation:       " << max_error << std::endl;

//     return 0;

    // 3. Analyze Output
    float max_mag = -1.0; // Initialize correctly
    int peak_bin = 0;

    std::cout << "Bin\tReal\tImag\tMagnitude" << std::endl;

    for (int i = 0; i < N; i++) {
    axis_pkt out_pkt = ip_to_dma.read();
        
        // Read the raw signed bits
        int16_t r_raw = (int16_t)out_pkt.data.range(15, 0);
        int16_t i_raw = (int16_t)out_pkt.data.range(31, 16);
        
        // Normalize to -1.0 ... 1.0 range
        float r_norm = (float)r_raw / 32768.0;
        float i_norm = (float)i_raw / 32768.0;
        
        float mag = sqrt(r_norm*r_norm + i_norm*i_norm);
        
        // Bins ~23 is correct for 1kHz @ 44.1ksps
        if (i > 20 && i < 26) {
            std::cout << "Bin " << i << ": \tMag: " << mag << std::endl;
        }
        


        // Look for the peak in the first half (Nyquist limit)
        if (i < N/2 && mag > max_mag && i!=0) {
            max_mag = mag;
            peak_bin = i;
        }
    }

    // 4. Verification
    float bin_width = fs / N;
    float detected_freq = peak_bin * bin_width;
    
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Target Frequency: " << freq << " Hz" << std::endl;
    std::cout << "Detected Peak: Bin " << peak_bin << " (" << detected_freq << " Hz)" << std::endl;

    // Check if detected frequency is within one bin of target
    if (std::abs(detected_freq - freq) <= bin_width) {
        std::cout << "RESULT: PASS" << std::endl;
        return 0;
    } else {
        std::cout << "RESULT: FAIL (Peak is at Bin " << peak_bin << ")" << std::endl;
        return 1;
    }


}