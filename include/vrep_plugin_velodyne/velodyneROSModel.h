#pragma once

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

class CVelodyneROSModel
{
public:
    CVelodyneROSModel(const int visionSensorHandles[4],float frequency,int options,float pointSize,float coloringDistances[2],float scalingFactor,int newPointCloudHandle, int local_frame_handle);
    virtual ~CVelodyneROSModel();

    int getVelodyneHandle();
    bool areVisionSensorsExplicitelyHandled();
    bool doAllObjectsExistAndAreVisionSensors();
    bool handle(float dt);
    void addPointsToBuffer(std::vector<float> &pts, sensor_msgs::PointCloud2 & buff);

private:
    void _removePointsBetween(float lowAngle,float range);
    void _getColorFromIntensity(float intensity,unsigned char col[3]);

    int _visionSensorHandles[4];
    float _frequency;
    int _velodyneHandle;
    float _displayScalingFactor;
    float _pointSize;
    bool _displayPts;
    bool _emissivePts;
    bool _displayOnlyCurrent;
    bool _cartesianCoords;
    int _ptCloudHandle;
    int _newPtCloudHandle;
    float lastScanAngle;
    float _coloringDistances[2];
    int _local_frame_handle;
    std::vector<float> _displayPtsXyz;
    std::vector<float> _displayPtsA;
    std::vector<unsigned char> _displayPtsCol;

    sensor_msgs::PointCloud2 _buffer;
    float _RANGE;
    ros::Publisher _pubVelodyne;

    static int _nextVelodyneHandle;
};
