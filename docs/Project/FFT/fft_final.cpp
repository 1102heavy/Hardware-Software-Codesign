/*
----------------------------------------------------------------------------------
--	(c) Rajesh C Panicker, NUS,
--  Description : AXI Stream Coprocessor (HLS), implementing the sum of 4 numbers
--	License terms :
--	You are free to use this code as long as you
--		(i) DO NOT post a modified version of this on any public repository;
--		(ii) use it only for educational purposes;
--		(iii) accept the responsibility to ensure that your implementation does not violate any intellectual property of any entity.
--		(iv) accept that the program is provided "as is" without warranty of any kind or assurance regarding its suitability for any particular purpose;
--		(v) send an email to rajesh.panicker@ieee.org briefly mentioning its use (except when used for the course EE4218/CEG5203 at the National University of Singapore);
--		(vi) retain this notice in this file or any files derived from this.
----------------------------------------------------------------------------------
*/

#include "hls_stream.h"
#include "ap_int.h"
#include "ap_axi_sdata.h"
#include <complex>
#include <ap_fixed.h>
#include <hls_math.h>


typedef ap_fixed<16, 1, AP_RND_CONV, AP_SAT> data_t;
typedef std::complex<data_t> complex_t;

#define N 1024
#define LOG2_N 10
typedef ap_axis<32, 0, 0, 0> axis_pkt; // Standard AXI structure


const complex_t twiddles[512] = {
    complex_t(1.000000, 0.000000),
    complex_t(0.999981, -0.006136),
    complex_t(0.999925, -0.012272),
    complex_t(0.999831, -0.018407),
    complex_t(0.999699, -0.024541),
    complex_t(0.999529, -0.030675),
    complex_t(0.999322, -0.036807),
    complex_t(0.999078, -0.042938),
    complex_t(0.998795, -0.049068),
    complex_t(0.998476, -0.055195),
    complex_t(0.998118, -0.061321),
    complex_t(0.997723, -0.067444),
    complex_t(0.997290, -0.073565),
    complex_t(0.996820, -0.079682),
    complex_t(0.996313, -0.085797),
    complex_t(0.995767, -0.091909),
    complex_t(0.995185, -0.098017),
    complex_t(0.994565, -0.104122),
    complex_t(0.993907, -0.110222),
    complex_t(0.993212, -0.116319),
    complex_t(0.992480, -0.122411),
    complex_t(0.991710, -0.128498),
    complex_t(0.990903, -0.134581),
    complex_t(0.990058, -0.140658),
    complex_t(0.989177, -0.146730),
    complex_t(0.988258, -0.152797),
    complex_t(0.987301, -0.158858),
    complex_t(0.986308, -0.164913),
    complex_t(0.985278, -0.170962),
    complex_t(0.984210, -0.177004),
    complex_t(0.983105, -0.183040),
    complex_t(0.981964, -0.189069),
    complex_t(0.980785, -0.195090),
    complex_t(0.979570, -0.201105),
    complex_t(0.978317, -0.207111),
    complex_t(0.977028, -0.213110),
    complex_t(0.975702, -0.219101),
    complex_t(0.974339, -0.225084),
    complex_t(0.972940, -0.231058),
    complex_t(0.971504, -0.237024),
    complex_t(0.970031, -0.242980),
    complex_t(0.968522, -0.248928),
    complex_t(0.966976, -0.254866),
    complex_t(0.965394, -0.260794),
    complex_t(0.963776, -0.266713),
    complex_t(0.962121, -0.272621),
    complex_t(0.960431, -0.278520),
    complex_t(0.958703, -0.284408),
    complex_t(0.956940, -0.290285),
    complex_t(0.955141, -0.296151),
    complex_t(0.953306, -0.302006),
    complex_t(0.951435, -0.307850),
    complex_t(0.949528, -0.313682),
    complex_t(0.947586, -0.319502),
    complex_t(0.945607, -0.325310),
    complex_t(0.943593, -0.331106),
    complex_t(0.941544, -0.336890),
    complex_t(0.939459, -0.342661),
    complex_t(0.937339, -0.348419),
    complex_t(0.935184, -0.354164),
    complex_t(0.932993, -0.359895),
    complex_t(0.930767, -0.365613),
    complex_t(0.928506, -0.371317),
    complex_t(0.926210, -0.377007),
    complex_t(0.923880, -0.382683),
    complex_t(0.921514, -0.388345),
    complex_t(0.919114, -0.393992),
    complex_t(0.916679, -0.399624),
    complex_t(0.914210, -0.405241),
    complex_t(0.911706, -0.410843),
    complex_t(0.909168, -0.416430),
    complex_t(0.906596, -0.422000),
    complex_t(0.903989, -0.427555),
    complex_t(0.901349, -0.433094),
    complex_t(0.898674, -0.438616),
    complex_t(0.895966, -0.444122),
    complex_t(0.893224, -0.449611),
    complex_t(0.890449, -0.455084),
    complex_t(0.887640, -0.460539),
    complex_t(0.884797, -0.465976),
    complex_t(0.881921, -0.471397),
    complex_t(0.879012, -0.476799),
    complex_t(0.876070, -0.482184),
    complex_t(0.873095, -0.487550),
    complex_t(0.870087, -0.492898),
    complex_t(0.867046, -0.498228),
    complex_t(0.863973, -0.503538),
    complex_t(0.860867, -0.508830),
    complex_t(0.857729, -0.514103),
    complex_t(0.854558, -0.519356),
    complex_t(0.851355, -0.524590),
    complex_t(0.848120, -0.529804),
    complex_t(0.844854, -0.534998),
    complex_t(0.841555, -0.540171),
    complex_t(0.838225, -0.545325),
    complex_t(0.834863, -0.550458),
    complex_t(0.831470, -0.555570),
    complex_t(0.828045, -0.560662),
    complex_t(0.824589, -0.565732),
    complex_t(0.821103, -0.570781),
    complex_t(0.817585, -0.575808),
    complex_t(0.814036, -0.580814),
    complex_t(0.810457, -0.585798),
    complex_t(0.806848, -0.590760),
    complex_t(0.803208, -0.595699),
    complex_t(0.799537, -0.600616),
    complex_t(0.795837, -0.605511),
    complex_t(0.792107, -0.610383),
    complex_t(0.788346, -0.615232),
    complex_t(0.784557, -0.620057),
    complex_t(0.780737, -0.624859),
    complex_t(0.776888, -0.629638),
    complex_t(0.773010, -0.634393),
    complex_t(0.769103, -0.639124),
    complex_t(0.765167, -0.643832),
    complex_t(0.761202, -0.648514),
    complex_t(0.757209, -0.653173),
    complex_t(0.753187, -0.657807),
    complex_t(0.749136, -0.662416),
    complex_t(0.745058, -0.667000),
    complex_t(0.740951, -0.671559),
    complex_t(0.736817, -0.676093),
    complex_t(0.732654, -0.680601),
    complex_t(0.728464, -0.685084),
    complex_t(0.724247, -0.689541),
    complex_t(0.720003, -0.693971),
    complex_t(0.715731, -0.698376),
    complex_t(0.711432, -0.702755),
    complex_t(0.707107, -0.707107),
    complex_t(0.702755, -0.711432),
    complex_t(0.698376, -0.715731),
    complex_t(0.693971, -0.720003),
    complex_t(0.689541, -0.724247),
    complex_t(0.685084, -0.728464),
    complex_t(0.680601, -0.732654),
    complex_t(0.676093, -0.736817),
    complex_t(0.671559, -0.740951),
    complex_t(0.667000, -0.745058),
    complex_t(0.662416, -0.749136),
    complex_t(0.657807, -0.753187),
    complex_t(0.653173, -0.757209),
    complex_t(0.648514, -0.761202),
    complex_t(0.643832, -0.765167),
    complex_t(0.639124, -0.769103),
    complex_t(0.634393, -0.773010),
    complex_t(0.629638, -0.776888),
    complex_t(0.624859, -0.780737),
    complex_t(0.620057, -0.784557),
    complex_t(0.615232, -0.788346),
    complex_t(0.610383, -0.792107),
    complex_t(0.605511, -0.795837),
    complex_t(0.600616, -0.799537),
    complex_t(0.595699, -0.803208),
    complex_t(0.590760, -0.806848),
    complex_t(0.585798, -0.810457),
    complex_t(0.580814, -0.814036),
    complex_t(0.575808, -0.817585),
    complex_t(0.570781, -0.821103),
    complex_t(0.565732, -0.824589),
    complex_t(0.560662, -0.828045),
    complex_t(0.555570, -0.831470),
    complex_t(0.550458, -0.834863),
    complex_t(0.545325, -0.838225),
    complex_t(0.540171, -0.841555),
    complex_t(0.534998, -0.844854),
    complex_t(0.529804, -0.848120),
    complex_t(0.524590, -0.851355),
    complex_t(0.519356, -0.854558),
    complex_t(0.514103, -0.857729),
    complex_t(0.508830, -0.860867),
    complex_t(0.503538, -0.863973),
    complex_t(0.498228, -0.867046),
    complex_t(0.492898, -0.870087),
    complex_t(0.487550, -0.873095),
    complex_t(0.482184, -0.876070),
    complex_t(0.476799, -0.879012),
    complex_t(0.471397, -0.881921),
    complex_t(0.465976, -0.884797),
    complex_t(0.460539, -0.887640),
    complex_t(0.455084, -0.890449),
    complex_t(0.449611, -0.893224),
    complex_t(0.444122, -0.895966),
    complex_t(0.438616, -0.898674),
    complex_t(0.433094, -0.901349),
    complex_t(0.427555, -0.903989),
    complex_t(0.422000, -0.906596),
    complex_t(0.416430, -0.909168),
    complex_t(0.410843, -0.911706),
    complex_t(0.405241, -0.914210),
    complex_t(0.399624, -0.916679),
    complex_t(0.393992, -0.919114),
    complex_t(0.388345, -0.921514),
    complex_t(0.382683, -0.923880),
    complex_t(0.377007, -0.926210),
    complex_t(0.371317, -0.928506),
    complex_t(0.365613, -0.930767),
    complex_t(0.359895, -0.932993),
    complex_t(0.354164, -0.935184),
    complex_t(0.348419, -0.937339),
    complex_t(0.342661, -0.939459),
    complex_t(0.336890, -0.941544),
    complex_t(0.331106, -0.943593),
    complex_t(0.325310, -0.945607),
    complex_t(0.319502, -0.947586),
    complex_t(0.313682, -0.949528),
    complex_t(0.307850, -0.951435),
    complex_t(0.302006, -0.953306),
    complex_t(0.296151, -0.955141),
    complex_t(0.290285, -0.956940),
    complex_t(0.284408, -0.958703),
    complex_t(0.278520, -0.960431),
    complex_t(0.272621, -0.962121),
    complex_t(0.266713, -0.963776),
    complex_t(0.260794, -0.965394),
    complex_t(0.254866, -0.966976),
    complex_t(0.248928, -0.968522),
    complex_t(0.242980, -0.970031),
    complex_t(0.237024, -0.971504),
    complex_t(0.231058, -0.972940),
    complex_t(0.225084, -0.974339),
    complex_t(0.219101, -0.975702),
    complex_t(0.213110, -0.977028),
    complex_t(0.207111, -0.978317),
    complex_t(0.201105, -0.979570),
    complex_t(0.195090, -0.980785),
    complex_t(0.189069, -0.981964),
    complex_t(0.183040, -0.983105),
    complex_t(0.177004, -0.984210),
    complex_t(0.170962, -0.985278),
    complex_t(0.164913, -0.986308),
    complex_t(0.158858, -0.987301),
    complex_t(0.152797, -0.988258),
    complex_t(0.146730, -0.989177),
    complex_t(0.140658, -0.990058),
    complex_t(0.134581, -0.990903),
    complex_t(0.128498, -0.991710),
    complex_t(0.122411, -0.992480),
    complex_t(0.116319, -0.993212),
    complex_t(0.110222, -0.993907),
    complex_t(0.104122, -0.994565),
    complex_t(0.098017, -0.995185),
    complex_t(0.091909, -0.995767),
    complex_t(0.085797, -0.996313),
    complex_t(0.079682, -0.996820),
    complex_t(0.073565, -0.997290),
    complex_t(0.067444, -0.997723),
    complex_t(0.061321, -0.998118),
    complex_t(0.055195, -0.998476),
    complex_t(0.049068, -0.998795),
    complex_t(0.042938, -0.999078),
    complex_t(0.036807, -0.999322),
    complex_t(0.030675, -0.999529),
    complex_t(0.024541, -0.999699),
    complex_t(0.018407, -0.999831),
    complex_t(0.012272, -0.999925),
    complex_t(0.006136, -0.999981),
    complex_t(0.000000, -1.000000),
    complex_t(-0.006136, -0.999981),
    complex_t(-0.012272, -0.999925),
    complex_t(-0.018407, -0.999831),
    complex_t(-0.024541, -0.999699),
    complex_t(-0.030675, -0.999529),
    complex_t(-0.036807, -0.999322),
    complex_t(-0.042938, -0.999078),
    complex_t(-0.049068, -0.998795),
    complex_t(-0.055195, -0.998476),
    complex_t(-0.061321, -0.998118),
    complex_t(-0.067444, -0.997723),
    complex_t(-0.073565, -0.997290),
    complex_t(-0.079682, -0.996820),
    complex_t(-0.085797, -0.996313),
    complex_t(-0.091909, -0.995767),
    complex_t(-0.098017, -0.995185),
    complex_t(-0.104122, -0.994565),
    complex_t(-0.110222, -0.993907),
    complex_t(-0.116319, -0.993212),
    complex_t(-0.122411, -0.992480),
    complex_t(-0.128498, -0.991710),
    complex_t(-0.134581, -0.990903),
    complex_t(-0.140658, -0.990058),
    complex_t(-0.146730, -0.989177),
    complex_t(-0.152797, -0.988258),
    complex_t(-0.158858, -0.987301),
    complex_t(-0.164913, -0.986308),
    complex_t(-0.170962, -0.985278),
    complex_t(-0.177004, -0.984210),
    complex_t(-0.183040, -0.983105),
    complex_t(-0.189069, -0.981964),
    complex_t(-0.195090, -0.980785),
    complex_t(-0.201105, -0.979570),
    complex_t(-0.207111, -0.978317),
    complex_t(-0.213110, -0.977028),
    complex_t(-0.219101, -0.975702),
    complex_t(-0.225084, -0.974339),
    complex_t(-0.231058, -0.972940),
    complex_t(-0.237024, -0.971504),
    complex_t(-0.242980, -0.970031),
    complex_t(-0.248928, -0.968522),
    complex_t(-0.254866, -0.966976),
    complex_t(-0.260794, -0.965394),
    complex_t(-0.266713, -0.963776),
    complex_t(-0.272621, -0.962121),
    complex_t(-0.278520, -0.960431),
    complex_t(-0.284408, -0.958703),
    complex_t(-0.290285, -0.956940),
    complex_t(-0.296151, -0.955141),
    complex_t(-0.302006, -0.953306),
    complex_t(-0.307850, -0.951435),
    complex_t(-0.313682, -0.949528),
    complex_t(-0.319502, -0.947586),
    complex_t(-0.325310, -0.945607),
    complex_t(-0.331106, -0.943593),
    complex_t(-0.336890, -0.941544),
    complex_t(-0.342661, -0.939459),
    complex_t(-0.348419, -0.937339),
    complex_t(-0.354164, -0.935184),
    complex_t(-0.359895, -0.932993),
    complex_t(-0.365613, -0.930767),
    complex_t(-0.371317, -0.928506),
    complex_t(-0.377007, -0.926210),
    complex_t(-0.382683, -0.923880),
    complex_t(-0.388345, -0.921514),
    complex_t(-0.393992, -0.919114),
    complex_t(-0.399624, -0.916679),
    complex_t(-0.405241, -0.914210),
    complex_t(-0.410843, -0.911706),
    complex_t(-0.416430, -0.909168),
    complex_t(-0.422000, -0.906596),
    complex_t(-0.427555, -0.903989),
    complex_t(-0.433094, -0.901349),
    complex_t(-0.438616, -0.898674),
    complex_t(-0.444122, -0.895966),
    complex_t(-0.449611, -0.893224),
    complex_t(-0.455084, -0.890449),
    complex_t(-0.460539, -0.887640),
    complex_t(-0.465976, -0.884797),
    complex_t(-0.471397, -0.881921),
    complex_t(-0.476799, -0.879012),
    complex_t(-0.482184, -0.876070),
    complex_t(-0.487550, -0.873095),
    complex_t(-0.492898, -0.870087),
    complex_t(-0.498228, -0.867046),
    complex_t(-0.503538, -0.863973),
    complex_t(-0.508830, -0.860867),
    complex_t(-0.514103, -0.857729),
    complex_t(-0.519356, -0.854558),
    complex_t(-0.524590, -0.851355),
    complex_t(-0.529804, -0.848120),
    complex_t(-0.534998, -0.844854),
    complex_t(-0.540171, -0.841555),
    complex_t(-0.545325, -0.838225),
    complex_t(-0.550458, -0.834863),
    complex_t(-0.555570, -0.831470),
    complex_t(-0.560662, -0.828045),
    complex_t(-0.565732, -0.824589),
    complex_t(-0.570781, -0.821103),
    complex_t(-0.575808, -0.817585),
    complex_t(-0.580814, -0.814036),
    complex_t(-0.585798, -0.810457),
    complex_t(-0.590760, -0.806848),
    complex_t(-0.595699, -0.803208),
    complex_t(-0.600616, -0.799537),
    complex_t(-0.605511, -0.795837),
    complex_t(-0.610383, -0.792107),
    complex_t(-0.615232, -0.788346),
    complex_t(-0.620057, -0.784557),
    complex_t(-0.624859, -0.780737),
    complex_t(-0.629638, -0.776888),
    complex_t(-0.634393, -0.773010),
    complex_t(-0.639124, -0.769103),
    complex_t(-0.643832, -0.765167),
    complex_t(-0.648514, -0.761202),
    complex_t(-0.653173, -0.757209),
    complex_t(-0.657807, -0.753187),
    complex_t(-0.662416, -0.749136),
    complex_t(-0.667000, -0.745058),
    complex_t(-0.671559, -0.740951),
    complex_t(-0.676093, -0.736817),
    complex_t(-0.680601, -0.732654),
    complex_t(-0.685084, -0.728464),
    complex_t(-0.689541, -0.724247),
    complex_t(-0.693971, -0.720003),
    complex_t(-0.698376, -0.715731),
    complex_t(-0.702755, -0.711432),
    complex_t(-0.707107, -0.707107),
    complex_t(-0.711432, -0.702755),
    complex_t(-0.715731, -0.698376),
    complex_t(-0.720003, -0.693971),
    complex_t(-0.724247, -0.689541),
    complex_t(-0.728464, -0.685084),
    complex_t(-0.732654, -0.680601),
    complex_t(-0.736817, -0.676093),
    complex_t(-0.740951, -0.671559),
    complex_t(-0.745058, -0.667000),
    complex_t(-0.749136, -0.662416),
    complex_t(-0.753187, -0.657807),
    complex_t(-0.757209, -0.653173),
    complex_t(-0.761202, -0.648514),
    complex_t(-0.765167, -0.643832),
    complex_t(-0.769103, -0.639124),
    complex_t(-0.773010, -0.634393),
    complex_t(-0.776888, -0.629638),
    complex_t(-0.780737, -0.624859),
    complex_t(-0.784557, -0.620057),
    complex_t(-0.788346, -0.615232),
    complex_t(-0.792107, -0.610383),
    complex_t(-0.795837, -0.605511),
    complex_t(-0.799537, -0.600616),
    complex_t(-0.803208, -0.595699),
    complex_t(-0.806848, -0.590760),
    complex_t(-0.810457, -0.585798),
    complex_t(-0.814036, -0.580814),
    complex_t(-0.817585, -0.575808),
    complex_t(-0.821103, -0.570781),
    complex_t(-0.824589, -0.565732),
    complex_t(-0.828045, -0.560662),
    complex_t(-0.831470, -0.555570),
    complex_t(-0.834863, -0.550458),
    complex_t(-0.838225, -0.545325),
    complex_t(-0.841555, -0.540171),
    complex_t(-0.844854, -0.534998),
    complex_t(-0.848120, -0.529804),
    complex_t(-0.851355, -0.524590),
    complex_t(-0.854558, -0.519356),
    complex_t(-0.857729, -0.514103),
    complex_t(-0.860867, -0.508830),
    complex_t(-0.863973, -0.503538),
    complex_t(-0.867046, -0.498228),
    complex_t(-0.870087, -0.492898),
    complex_t(-0.873095, -0.487550),
    complex_t(-0.876070, -0.482184),
    complex_t(-0.879012, -0.476799),
    complex_t(-0.881921, -0.471397),
    complex_t(-0.884797, -0.465976),
    complex_t(-0.887640, -0.460539),
    complex_t(-0.890449, -0.455084),
    complex_t(-0.893224, -0.449611),
    complex_t(-0.895966, -0.444122),
    complex_t(-0.898674, -0.438616),
    complex_t(-0.901349, -0.433094),
    complex_t(-0.903989, -0.427555),
    complex_t(-0.906596, -0.422000),
    complex_t(-0.909168, -0.416430),
    complex_t(-0.911706, -0.410843),
    complex_t(-0.914210, -0.405241),
    complex_t(-0.916679, -0.399624),
    complex_t(-0.919114, -0.393992),
    complex_t(-0.921514, -0.388345),
    complex_t(-0.923880, -0.382683),
    complex_t(-0.926210, -0.377007),
    complex_t(-0.928506, -0.371317),
    complex_t(-0.930767, -0.365613),
    complex_t(-0.932993, -0.359895),
    complex_t(-0.935184, -0.354164),
    complex_t(-0.937339, -0.348419),
    complex_t(-0.939459, -0.342661),
    complex_t(-0.941544, -0.336890),
    complex_t(-0.943593, -0.331106),
    complex_t(-0.945607, -0.325310),
    complex_t(-0.947586, -0.319502),
    complex_t(-0.949528, -0.313682),
    complex_t(-0.951435, -0.307850),
    complex_t(-0.953306, -0.302006),
    complex_t(-0.955141, -0.296151),
    complex_t(-0.956940, -0.290285),
    complex_t(-0.958703, -0.284408),
    complex_t(-0.960431, -0.278520),
    complex_t(-0.962121, -0.272621),
    complex_t(-0.963776, -0.266713),
    complex_t(-0.965394, -0.260794),
    complex_t(-0.966976, -0.254866),
    complex_t(-0.968522, -0.248928),
    complex_t(-0.970031, -0.242980),
    complex_t(-0.971504, -0.237024),
    complex_t(-0.972940, -0.231058),
    complex_t(-0.974339, -0.225084),
    complex_t(-0.975702, -0.219101),
    complex_t(-0.977028, -0.213110),
    complex_t(-0.978317, -0.207111),
    complex_t(-0.979570, -0.201105),
    complex_t(-0.980785, -0.195090),
    complex_t(-0.981964, -0.189069),
    complex_t(-0.983105, -0.183040),
    complex_t(-0.984210, -0.177004),
    complex_t(-0.985278, -0.170962),
    complex_t(-0.986308, -0.164913),
    complex_t(-0.987301, -0.158858),
    complex_t(-0.988258, -0.152797),
    complex_t(-0.989177, -0.146730),
    complex_t(-0.990058, -0.140658),
    complex_t(-0.990903, -0.134581),
    complex_t(-0.991710, -0.128498),
    complex_t(-0.992480, -0.122411),
    complex_t(-0.993212, -0.116319),
    complex_t(-0.993907, -0.110222),
    complex_t(-0.994565, -0.104122),
    complex_t(-0.995185, -0.098017),
    complex_t(-0.995767, -0.091909),
    complex_t(-0.996313, -0.085797),
    complex_t(-0.996820, -0.079682),
    complex_t(-0.997290, -0.073565),
    complex_t(-0.997723, -0.067444),
    complex_t(-0.998118, -0.061321),
    complex_t(-0.998476, -0.055195),
    complex_t(-0.998795, -0.049068),
    complex_t(-0.999078, -0.042938),
    complex_t(-0.999322, -0.036807),
    complex_t(-0.999529, -0.030675),
    complex_t(-0.999699, -0.024541),
    complex_t(-0.999831, -0.018407),
    complex_t(-0.999925, -0.012272),
    complex_t(-0.999981, -0.006136)
};

// --- FFT Logic: Double-Buffered, Reshaped, and Pipelined ---
void my_fft_logic(data_t data_re[N], data_t data_im[N]) {
    // Ping-pong workspace to avoid in-place dependency bottlenecks
    data_t work_re[N], work_im[N];
    
    // Partitioning: 2 banks ensures Lower and Upper indices are in different BRAMs
    #pragma HLS ARRAY_PARTITION variable=data_re cyclic factor=2 dim=1
    #pragma HLS ARRAY_PARTITION variable=data_im cyclic factor=2 dim=1
    #pragma HLS ARRAY_PARTITION variable=work_re cyclic factor=2 dim=1
    #pragma HLS ARRAY_PARTITION variable=work_im cyclic factor=2 dim=1

    // Partition Twiddles to prevent ROM read bottlenecks
    #pragma HLS ARRAY_PARTITION variable=twiddles cyclic factor=4 dim=1

    Stage_Pairs: for (int stage_odd = 1; stage_odd <= LOG2_N; stage_odd += 2) {
        
        // --- ODD STAGE: Read from 'data' -> Write to 'work' ---
        Butterfly_Odd: for (int i = 0; i < N/2; i++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS DEPENDENCE variable=work_re inter false
            #pragma HLS DEPENDENCE variable=work_im inter false
            #pragma HLS DEPENDENCE variable=data_re inter false
            #pragma HLS DEPENDENCE variable=data_im inter false
            int points = 1 << stage_odd;
            int half = points >> 1;
            int mask = half - 1;
            int l_idx = ((i >> (stage_odd - 1)) << stage_odd) | (i & mask);
            int u_idx = l_idx | half;
            
            // Extract Twiddle components
            complex_t W = twiddles[(i & mask) << (LOG2_N - stage_odd)];
            data_t wr = W.real();
            data_t wi = W.imag();
            
            data_t ur = data_re[u_idx];
            data_t ui = data_im[u_idx];
            
            // Manual Complex Multiply: (ur + j*ui) * (wr + j*wi)
            data_t tmp_re = (ur * wr) - (ui * wi);
            data_t tmp_im = (ur * wi) + (ui * wr);
            
            data_t lr = data_re[l_idx];
            data_t li = data_im[l_idx];

            work_re[l_idx] = (lr + tmp_re) >> 1;
            work_im[l_idx] = (li + tmp_im) >> 1;
            work_re[u_idx] = (lr - tmp_re) >> 1;
            work_im[u_idx] = (li - tmp_im) >> 1;
        }

        // --- EVEN STAGE: Read from 'work' -> Write to 'data' ---
        int stage_even = stage_odd + 1;
        Butterfly_Even: for (int i = 0; i < N/2; i++) {
            #pragma HLS PIPELINE II=1
            #pragma HLS DEPENDENCE variable=work_re inter false
            #pragma HLS DEPENDENCE variable=work_im inter false
            #pragma HLS DEPENDENCE variable=data_re inter false
            #pragma HLS DEPENDENCE variable=data_im inter false
            int points = 1 << stage_even;
            int half = points >> 1;
            int mask = half - 1;
            int l_idx = ((i >> (stage_even - 1)) << stage_even) | (i & mask);
            int u_idx = l_idx | half;
            
            complex_t W = twiddles[(i & mask) << (LOG2_N - stage_even)];
            data_t wr = W.real();
            data_t wi = W.imag();
            
            data_t ur = work_re[u_idx];
            data_t ui = work_im[u_idx];
            
            data_t tmp_re = (ur * wr) - (ui * wi);
            data_t tmp_im = (ur * wi) + (ui * wr);
            
            data_t lr = work_re[l_idx];
            data_t li = work_im[l_idx];

            data_re[l_idx] = (lr + tmp_re) >> 1;
            data_im[l_idx] = (li + tmp_im) >> 1;
            data_re[u_idx] = (lr - tmp_re) >> 1;
            data_im[u_idx] = (li - tmp_im) >> 1;
        }
    }
}
// This struct defines a standard AXI-Stream packet for 32-bit data

void my_fft_only_dma_ip(
    hls::stream<axis_pkt>& data_in,   // From DMA (Memory-to-Stream)
    hls::stream<axis_pkt>& data_out   // To DMA (Stream-to-Memory)
) {
    #pragma HLS INTERFACE axis port=data_in
    #pragma HLS INTERFACE axis port=data_out
    #pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS

    // Split internal buffers for max throughput
    data_t buf_re[N], buf_im[N];
    #pragma HLS ARRAY_PARTITION variable=buf_re cyclic factor=2 dim=1
    #pragma HLS ARRAY_PARTITION variable=buf_im cyclic factor=2 dim=1

    // 1. Bit-Reversed Stream In
    Read_Loop: for (int i = 0; i < N; i++) {
        #pragma HLS PIPELINE II=1
        axis_pkt pkt = data_in.read();
        
        ap_uint<LOG2_N> j = 0;
        for (int bit = 0; bit < LOG2_N; bit++) {
            #pragma HLS UNROLL
            j[LOG2_N - 1 - bit] = ((ap_uint<LOG2_N>)i)[bit];
        }

        // Split 32-bit DMA word into Real and Imaginary components
        data_t in_re, in_im;
        in_re.range(15, 0) = pkt.data.range(15, 0);
        in_im.range(15, 0) = pkt.data.range(31, 16);
        
        buf_re[(int)j] = in_re;
        buf_im[(int)j] = in_im;
    }

    // 2. Execute FFT
    my_fft_logic(buf_re, buf_im);

    // 3. Bit-Reversed Stream Out (Optional: Remove 'j' logic if you want natural order)
    Write_Loop: for (int i = 0; i < N; i++) {
        #pragma HLS PIPELINE II=1
        axis_pkt out_pkt;
        
        out_pkt.data.range(15, 0)  = buf_re[i].range(15, 0);
        out_pkt.data.range(31, 16) = buf_im[i].range(15, 0);
        
        out_pkt.keep = -1;
        out_pkt.last = (i == N - 1) ? 1 : 0;
        data_out.write(out_pkt);
    }
}