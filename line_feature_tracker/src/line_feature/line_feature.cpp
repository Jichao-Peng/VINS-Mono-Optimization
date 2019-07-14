//
// Created by leo on 19-7-14.
//

#include "line_feature.h"

//线特征提取
void LineFeature::detectLineFeatures( cv::Mat img, vector<cv::line_descriptor::KeyLine> &lines, cv::Mat &ldesc, double min_line_length )
{

    // Detect line features
    lines.clear();
    cv::Ptr<cv::line_descriptor::BinaryDescriptor>   lbd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    if( Config::hasLines() )//是否使用线分割
    {

        if( !Config::useFLDLines() )//是否使用FLD直线
        {
            cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd = cv::line_descriptor::LSDDetectorC::createLSDDetectorC();
            // lsd parameters
            cv::line_descriptor::LSDDetectorC::LSDOptions opts;
            opts.refine       = Config::lsdRefine();
            opts.scale        = Config::lsdScale();
            opts.sigma_scale  = Config::lsdSigmaScale();
            opts.quant        = Config::lsdQuant();
            opts.ang_th       = Config::lsdAngTh();
            opts.log_eps      = Config::lsdLogEps();
            opts.density_th   = Config::lsdDensityTh();
            opts.n_bins       = Config::lsdNBins();
            opts.min_length   = min_line_length;
            lsd->detect( img, lines, Config::lsdScale(), 1, opts);
            // filter lines
            if( lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
            {
                // sort lines by their response
                sort( lines.begin(), lines.end(), sort_lines_by_response() );
                //sort( lines.begin(), lines.end(), sort_lines_by_length() );
                lines.resize(Config::lsdNFeatures());
                // reassign index
                for( int i = 0; i < Config::lsdNFeatures(); i++  )
                    lines[i].class_id = i;
            }
            lbd->compute( img, lines, ldesc);
        }
        else//这里应该使用的就是LSD直线检测+LBD特征描述
        {
            cv::Mat fld_img, img_gray;
            vector<cv::Vec4f> fld_lines;

            if( img.channels() != 1 )
            {
                cv::cvtColor( img, img_gray, CV_RGB2GRAY );
                img_gray.convertTo( fld_img, CV_8UC1 );
            }
            else
                img.convertTo( fld_img, CV_8UC1 );

            Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(min_line_length);
            fld->detect( fld_img, fld_lines );

            // filter lines
            if( fld_lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )//对线条进行滤波
            {
                // sort lines by their response
                sort( fld_lines.begin(), fld_lines.end(), sort_flines_by_length() );
                fld_lines.resize(Config::lsdNFeatures());
            }

            // loop over lines object transforming into a vector<KeyLine>
            lines.reserve(fld_lines.size());
            for( int i = 0; i < fld_lines.size(); i++ )
            {
                KeyLine kl;
                double octaveScale = 1.f;
                int    octaveIdx   = 0;

                kl.startPointX     = fld_lines[i][0] * octaveScale;
                kl.startPointY     = fld_lines[i][1] * octaveScale;
                kl.endPointX       = fld_lines[i][2] * octaveScale;
                kl.endPointY       = fld_lines[i][3] * octaveScale;

                kl.sPointInOctaveX = fld_lines[i][0];
                kl.sPointInOctaveY = fld_lines[i][1];
                kl.ePointInOctaveX = fld_lines[i][2];
                kl.ePointInOctaveY = fld_lines[i][3];

                kl.lineLength = (float) sqrt( pow( fld_lines[i][0] - fld_lines[i][2], 2 ) + pow( fld_lines[i][1] - fld_lines[i][3], 2 ) );

                kl.angle    = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
                kl.class_id = i;
                kl.octave   = octaveIdx;
                kl.size     = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
                kl.pt       = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

                kl.response = kl.lineLength / max( fld_img.cols, fld_img.rows );
                cv::LineIterator li( fld_img, Point2f( fld_lines[i][0], fld_lines[i][1] ), Point2f( fld_lines[i][2], fld_lines[i][3] ) );
                kl.numOfPixels = li.count;

                lines.push_back( kl );

            }

            // compute lbd descriptor
            lbd->compute( fld_img, lines, ldesc);
        }

    }
}

//线特征匹配
void LineFeature::matchLineFeatures(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<int> &matches)
{
    StVO::match(desc1, desc2, Config::minRatio12L(), matches);
}