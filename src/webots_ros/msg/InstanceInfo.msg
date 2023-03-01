# represents instance segmentation reult
# i-th instance has instance id as instance_id[i], class id as class_id[i], class name as class_names[i], scores as scores[i] and feature as features[i]
# instance_mask is a matrix with same size as the camera image, whose content is the instance id of each pixel

Header header
uint16[] instance_ids
uint16[] class_ids
string[] class_names
float32[] scores
feature[] features
sensor_msgs/Image instance_mask
