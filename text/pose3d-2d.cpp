#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
using namespace std;
using namespace cv;

 bool ComputeORB(Mat image, vector<KeyPoint> &keypoint_1,Mat & description_1)
 {
        const int half_patch_size = 8;
        const int half_boundary = 16;
        int bad_point = 0;
         //判断keypoints是否在图片内部
         for(auto kp : keypoint_1)
         {
             if(kp.pt.x < )
         }

 }

 
int main(int argc, char *argv[])
{
  if(argc != 3)
  {
      cout << "不够参数" << endl;
  }
  Mat image = imread(argv[1] ,1);
  Mat image2 = imread(argv[2], 1);
  vector<KeyPoint> keypoint_1, keypoint_2;
  vector<DMatch> matchs;
  Mat description_1 , description_2;
   
   Ptr<FeatureDetector> detector = ORB::create(); 
   Ptr<DescriptorExtractor> description = ORB::create();
   Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
   detector->detect(image , keypoint_1);
   detector->detect(image2 , keypoint_2);

//    description->compute(image , keypoint_1, description_1);
//    description->compute(image2 , keypoint_2, description_2);
    //自己写匹配描述子
     ComputeORB(image, keypoint_1, description_1);
     ComputeORB(image2, keypoint_2, description_2);

    matcher->match(description_1, description_2 , matchs);
    cout << "图片1的keypoints" << keypoint_1.size() << endl;
      cout << "图片2的keypoints" << keypoint_2.size() << endl;
    cout << "描述子的个数 " << description_2.size() << endl;
    cout << "匹配的点数" << matchs.size() << endl;
    /*
    InputArray image, const std::vector<KeyPoint>& keypoints, InputOutputArray outImage,
        const Scalar& color=Scalar::all(-1), int flags=DrawMatchesFlags::DEFAULT
    */
    drawKeypoints(image , keypoint_1 , image ,   Scalar::all(-1), DrawMatchesFlags::DRAW_OVER_OUTIMG);
    // for(int i = 0; i < keypoint_2.size(); i++)
    // {
    //     cout << keypoint_2[i].pt.x << "-" << keypoint_2[i].pt.y << " " ;
    // }
    for(int i = 0; i < matchs.size(); i++)
    {
       // cout << "输出第" <<  i << "个匹配点的距离" << endl;
      //  cout << matchs[i].distance << " " ;
    }
   cout << endl;

   double min_dist = 1000, max_dist = 0;
   //高
   for(int i = 0; i < description_1.rows ; i++)
   {
       double dist = matchs[i].distance;
        if(dist < min_dist)
       {
           min_dist = dist;
       }
       if(dist > max_dist)
       {
           max_dist = dist;  //全部的最大
       }
   
   }
   cout << "最大" << max_dist << endl;
   cout << "最小" << min_dist << endl;

vector<DMatch> bestmatch;
for(int i =0 ; i < description_2.rows ; i++)
{
    double dist  = matchs[i].distance;
    if(dist <= (2*min_dist  > 30 ? 2*min_dist : 30))
    {
        bestmatch.push_back(matchs[i]);
    }
}

cout << "bestmatch size = " << bestmatch.size() << endl;


  Mat img_match;
  Mat img_goodmatch;
  drawMatches(image, keypoint_1, image2, keypoint_2, matchs, img_match);
 // drawMatches(image2, keypoint_1, img_2, keypoint_2, good_matches, img_goodmatch);


  imshow("1" ,image);
  imshow("2", image2);
  imshow("333", img_match);
  waitKey(0);
  
    return 0;
}