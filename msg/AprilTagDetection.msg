# msg/AprilTagDetection.msg
string family
int32 id
float64 hamming
float64 decision_margin
float64[9] pose_R
float64[3] pose_t
float64 pose_err
float64[2] center
float64[8] corners      # Corners in image coordinates (x,y for each corner)