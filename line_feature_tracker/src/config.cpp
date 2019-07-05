/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#include "config.h"

//STL
#include <iostream>

//Boost
#include <boost/filesystem.hpp>

//YAML
//#include <yaml.h>

using namespace std;

Config::Config()
{

    // kf decision (SLAM) parameters
    min_entropy_ratio     = 0.85;
    max_kf_t_dist         = 5.0;
    max_kf_r_dist         = 15.0;

    // StVO-PL options
    // -----------------------------------------------------------------------------------------------------
    has_points         = true;      // true if using points
    has_lines          = true;      // true if using line segments
    use_fld_lines      = false;     // true if using FLD detector
    lr_in_parallel     = true;      // true if detecting and matching features in parallel
    pl_in_parallel     = true;      // true if detecting points and line segments in parallel
    best_lr_matches    = true;      // true if double-checking the matches between the two images
    adaptative_fast    = true;      // true if using adaptative fast_threshold
    use_motion_model   = false;     // true if using constant motion model

    // Tracking parameters
    // -----------------------------------------------------------------------------------------------------
    // Point features
    max_dist_epip     = 1.0;        // max. epipolar distance in pixels
    min_disp          = 1.0;        // min. disparity (avoid points in the infinite)
    min_ratio_12_p    = 0.9;        // min. ratio between the first and second best matches

    // Line segment features
    line_sim_th       = 0.75;       // threshold for cosine similarity
    stereo_overlap_th = 0.75;
    f2f_overlap_th    = 0.75;
    min_line_length   = 0.025;      // min. line length (relative to img size)
    line_horiz_th     = 0.1;        // parameter to avoid horizontal lines (pixels)
    min_ratio_12_l    = 0.9;        // parameter to avoid outliers in line matching
    ls_min_disp_ratio = 0.7;        // min ratio between min(disp)/max(disp) for a LS

    // Adaptative FAST parameters
    fast_min_th       = 5;          // min. value for FAST threshold
    fast_max_th       = 50;         // max. value for FAST threshold
    fast_inc_th       = 5;          // base increment for the FAST threshold
    fast_feat_th      = 50;         // base number of features to increase/decrease FAST threshold
    fast_err_th       = 0.5;        // threshold for the optimization error

    // Optimization parameters
    // -----------------------------------------------------------------------------------------------------
    homog_th         = 1e-7;        // avoid points in the infinite
    min_features     = 10;          // min. number of features to perform StVO
    max_iters        = 5;           // max. number of iterations in the first stage of the optimization
    max_iters_ref    = 10;          // max. number of iterations in the refinement stage
    min_error        = 1e-7;        // min. error to stop the optimization
    min_error_change = 1e-7;        // min. error change to stop the optimization
    inlier_k         = 4.0;         // factor to discard outliers before the refinement stage

    // Feature detection parameters
    // -----------------------------------------------------------------------------------------------------
    matching_strategy = 0;          // 0 - pure descriptor based  |  1 - window based plus descriptor
    matching_s_ws     = 10;         // size of the windows (in pixels) to look for stereo matches (if matching_stereo=1)
    matching_f2f_ws   = 3;          // size of the windows (in pixels) to look for f2f matches

    // ORB detector
    orb_nfeatures     = 1200;       // number of ORB features to detect
    orb_scale_factor  = 1.2;        // pyramid decimation ratio for the ORB detector
    orb_nlevels       = 4;          // number of pyramid levels
    orb_edge_th       = 19;         // size of the border where the features are not detected
    orb_wta_k         = 2;          //  number of points that produce each element of the oriented BRIEF descriptor
    orb_score         = 1;          // 0 - HARRIS  |  1 - FAST
    orb_patch_size    = 31;         // size of the patch used by the oriented BRIEF descriptor.
    orb_fast_th       = 20;         // default FAST threshold
    // LSD parameters
    lsd_nfeatures     = 300;        // number of LSD lines detected (set to 0 if keeping all lines)
    lsd_refine        = 0;          // the way of refining or not the detected lines
    lsd_scale         = 1.2;        // scale of the image that will be used to find the lines
    lsd_sigma_scale   = 0.6;        // sigma for Gaussian filter
    lsd_quant         = 2.0;        // bound to the quantization error on the gradient norm
    lsd_ang_th        = 22.5;       // gradient angle tolerance in degrees
    lsd_log_eps       = 1.0;        // detection threshold (only for advanced refinement)
    lsd_density_th    = 0.6;        // minimal density of aligned region points in the enclosing rectangle
    lsd_n_bins        = 1024;       // number of bins in pseudo-ordering of gradient modulus
}

Config::~Config(){}

Config& Config::getInstance()
{
    static Config instance; // Instantiated on first use and guaranteed to be destroyed
    return instance;
}

//template<typename T>
//inline T loadSafe(const YAML::Node &config, std::string param, T default_value = T()) {
//
//    if (YAML::Node parameter = config[param])
//        return parameter.as<T>();
//    else
//        return default_value;
//}
//
//void Config::loadFromFile( const string &config_file )
//{
//
//    if (!boost::filesystem::exists(config_file) || !boost::filesystem::is_regular(config_file)) {
//        cout << "[Config->loadFromFile] Invalid config file, keeping default params..." << endl;
//        return;
//    }
//
//    YAML::Node config = YAML::LoadFile(config_file);
//
//    Config::minEntropyRatio() = loadSafe(config, "min_entropy_ratio", Config::minEntropyRatio());
//    Config::maxKFTDist() = loadSafe(config, "max_kf_t_dist", Config::maxKFTDist());
//    Config::maxKFRDist() = loadSafe(config, "max_kf_r_dist", Config::maxKFRDist());
//
//    Config::hasPoints() = loadSafe(config, "has_points", Config::hasPoints());
//    Config::hasLines() = loadSafe(config, "has_lines", Config::hasLines());
//    Config::useFLDLines() = loadSafe(config, "use_fld_lines", Config::useFLDLines());
//    Config::lrInParallel() = loadSafe(config, "lr_in_parallel", Config::lrInParallel());
//    Config::plInParallel() = loadSafe(config, "pl_in_parallel", Config::plInParallel());
//    Config::bestLRMatches() = loadSafe(config, "best_lr_matches", Config::bestLRMatches());
//    Config::adaptativeFAST() = loadSafe(config, "adaptative_fast", Config::adaptativeFAST());
//    Config::useMotionModel() = loadSafe(config, "use_motion_model", Config::useMotionModel());
//
//    Config::maxDistEpip() = loadSafe(config, "max_dist_epip", Config::maxDistEpip());
//    Config::minDisp() = loadSafe(config, "min_disp", Config::minDisp());
//    Config::minRatio12P() = loadSafe(config, "min_ratio_12_p", Config::minRatio12P());
//
//    Config::lineSimTh() = loadSafe(config, "line_sim_th", Config::lineSimTh());
//    Config::stereoOverlapTh() = loadSafe(config, "stereo_overlap_th", Config::stereoOverlapTh());
//    Config::f2fOverlapTh() = loadSafe(config, "f2f_overlap_th", Config::f2fOverlapTh());
//    Config::minLineLength() = loadSafe(config, "min_line_length", Config::minLineLength());
//    Config::lineHorizTh() = loadSafe(config, "line_horiz_th", Config::lineHorizTh());
//    Config::minRatio12L() = loadSafe(config, "min_ratio_12_l", Config::minRatio12L());
//    Config::lsMinDispRatio() = loadSafe(config, "ls_min_disp_ratio", Config::lsMinDispRatio());
//
//    Config::fastMinTh() = loadSafe(config, "fast_min_th", Config::fastMinTh());
//    Config::fastMaxTh() = loadSafe(config, "fast_max_th", Config::fastMaxTh());
//    Config::fastIncTh() = loadSafe(config, "fast_inc_th", Config::fastIncTh());
//    Config::fastFeatTh() = loadSafe(config, "fast_feat_th", Config::fastFeatTh());
//    Config::fastErrTh() = loadSafe(config, "fast_err_th", Config::fastErrTh());
//
//    Config::rgbdMinDepth() = loadSafe(config, "rgbd_min_depth", Config::rgbdMinDepth());
//    Config::rgbdMaxDepth() = loadSafe(config, "rgbd_max_depth", Config::rgbdMaxDepth());
//
//    Config::homogTh() = loadSafe(config, "homog_th", Config::homogTh());
//    Config::minFeatures() = loadSafe(config, "min_features", Config::minFeatures());
//    Config::maxIters() = loadSafe(config, "max_iters", Config::maxIters());
//    Config::maxItersRef() = loadSafe(config, "max_iters_ref", Config::maxItersRef());
//    Config::minError() = loadSafe(config, "min_error", Config::minError());
//    Config::minErrorChange() = loadSafe(config, "min_error_change", Config::minErrorChange());
//    Config::inlierK() = loadSafe(config, "inlier_k", Config::inlierK());
//
//    Config::matchingStrategy() = loadSafe(config, "matching_strategy", Config::matchingStrategy());
//    Config::matchingSWs() = loadSafe(config, "matching_s_ws", Config::matchingSWs());
//    Config::matchingF2FWs() = loadSafe(config, "matching_f2f_ws", Config::matchingF2FWs());
//
//    Config::orbNFeatures() = loadSafe(config, "orb_nfeatures", Config::orbNFeatures());
//    Config::orbScaleFactor() = loadSafe(config, "orb_scale_factor", Config::orbScaleFactor());
//    Config::orbNLevels() = loadSafe(config, "orb_nlevels", Config::orbNLevels());
//    Config::orbEdgeTh() = loadSafe(config, "orb_edge_th", Config::orbEdgeTh());
//    Config::orbWtaK() = loadSafe(config, "orb_wta_k", Config::orbWtaK());
//    Config::orbScore() = loadSafe(config, "orb_score", Config::orbScore());
//    Config::orbPatchSize() = loadSafe(config, "orb_patch_size", Config::orbPatchSize());
//    Config::orbFastTh() = loadSafe(config, "orb_fast_th", Config::orbFastTh());
//
//    Config::lsdNFeatures() = loadSafe(config, "lsd_nfeatures", Config::lsdNFeatures());
//    Config::lsdRefine() = loadSafe(config, "lsd_refine", Config::lsdRefine());
//    Config::lsdScale() = loadSafe(config, "lsd_scale", Config::lsdScale());
//    Config::lsdSigmaScale() = loadSafe(config, "lsd_sigma_scale", Config::lsdSigmaScale());
//    Config::lsdQuant() = loadSafe(config, "lsd_quant", Config::lsdQuant());
//    Config::lsdAngTh() = loadSafe(config, "lsd_ang_th", Config::lsdAngTh());
//    Config::lsdLogEps() = loadSafe(config, "lsd_log_eps", Config::lsdLogEps());
//    Config::lsdDensityTh() = loadSafe(config, "lsd_density_th", Config::lsdDensityTh());
//    Config::lsdNBins() = loadSafe(config, "lsd_n_bins", Config::lsdNBins());
//}
