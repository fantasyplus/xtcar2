import math

#该函数用于将经纬度转换为XY坐标值
def blh2xyz(lat,lon):
    lat_org = 32.0242100*math.pi/180.0
    lon_org = 118.89928833*math.pi/180.0
    lat = lat*math.pi/180.0
    lon = lon*math.pi/180.0
    x = (lat - lat_org) * 6378137
    y = (lon - lon_org) * 6378137 * math.cos(lat)
    return x,y

#该函数用于将XY坐标值转换为经纬度
def xyz2blh(x,y):
    pose = []
    lat_org = 32.0242100*math.pi/180.0
    lon_org = 118.89928833*math.pi/180.0
    lat = x/6378137+lat_org
    #print(lat)
    lat_degree = lat*180.0/math.pi
    lon = y/math.cos(lat)/6378137+lon_org
    lon_degree = lon*180.0/math.pi
    return lat_degree,lon_degree

#该函数用于将经纬度转换为相对a0的坐标值  32.03074847222222,118.93994230555556
#a0的XY坐标值：-758.4042297349142 -3833.952162197224
def blh2xyz_opp(lat,lon):
     #输入经纬度
    x,y = blh2xyz(lat,lon)
    #取相反数
    x = -x
    y = -y
    #print("XY坐标值：",x,y)
    #计算相对a0的坐标值
    x_opp = x+758.4042297349142
    y_opp = y+3833.952162197224
    print("相对a0的坐标值：",x_opp,y_opp)
    return x_opp,y_opp

if __name__=='__main__':

    x_opp,y_opp = blh2xyz_opp(32.03074847222222,118.93994230555556)
    "务必注意文件的保存、命名和区分"
    #./uwb_three/016/uwb_zcx_016.txt
    f = open("./xz/pose.txt",'r',encoding='utf-8')#/uwb_test_715/uwb_zcx_016_sta.txt
    # f = open("./7_21/2102.txt",'r',encoding='utf-8')#/uwb_test_715/uwb_zcx_016_sta.txt
    #./pose/016/pose_uwb_016.txt
    f1 = open('./xz/blhpose.txt','w')#./pose/test_715/pose_test2.txt
    for line in f:
        data = line.strip('\n').split('\t')
        
        x = float(data[1])
        # x = -x
        y = float(data[2])

        # z = float(data[2])

        # if(z == 4):
        # y = -y
        # print("x:",x)
        # print("y:",y)
        lat,lon = xyz2blh(x,y)
        # lat,lon = xyz2blh(y,x)
        pose = str(lon)+','+str(lat)+','+'0'+'\n'
        f1.write(pose)
    