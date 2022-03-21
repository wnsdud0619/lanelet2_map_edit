## required : ubuntu 18.04, ros-melodic   

## required topics
/sensing/camera/traffic_light/camera_info   
/sensing/camera/traffic_light/image_raw   
/tf   
/tf_static   
/map/vector_map   

## setting
map_edit_ws/src/edit_TL/config/edit_TL.yaml에서 AAP map_loader위도, 경도 확인   
https://www.latlong.net/lat-long-utm.html 사이트에서 위경도 입력하고 UTM Easting을 x에, UTM Northing을 y에 입력   
dir_path의 저장경로 확인   


## satart

	$ catkin_make
	$ bash start_map_based.sh
	$ bash start_edit_TL.sh


## 사용법
ESC : 종료   
Tab : roi변경(선택된 roi는 빨간색)   
a,s,d,w : roi 위치 이동   
space : 저장   

## 주의사항
1. roi는 정지상태에서 수정해야한다. 즉, bagfile을 정지해놓고 수정해야함.   
2. 최대한 정지선에서 정지하고 수정하기를 권장.   
3. roi 수정할때 마다 저장해줘야함.   


## test_bagfile
NAS120/TrafficLight/3_실험/220321_map_edit/map_test.bag 경로에 업로드(약5GB)





