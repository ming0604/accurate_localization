---
title: '#定位測試'

---

# 定位測試
## 一般步驟
**1.放檔案** 
將實驗錄的包含sensor data的bag檔放入..../lab_localization/src/navigation/amcl/amcl_data中以及地圖.pgm和.yaml放入/lab_localization/src/navigation/amcl/amcl_data/maps
>[!Note]
>.yaml 中的pgm檔路徑要改對

**2.跑定位** 
我主要程式在/src/==accurate_localization==內
accurate_localization/my_localize.launch設置：
1.若要後續做RMSE分析，要錄跑定位時的bag下來，在step1前先解開在 my_localize.launch最後面錄bag的註解，可改bag檔名及要錄的topics，不需要錄bag時也可註解掉這段
```
    <node pkg="rosbag" type="record" name="rosbag_record" args="-o /home/mingzhun/lab_localization/bag/2025_0119_3_openloop_PLICP_20241218_vive_mapping_test3_no_calid
            /tf
            /tf_static
            /base_gt_odom
            /base_gt
            /amcl_pose
            /PLICP_pose"
    /> 
```
2.設置要用的map
```
<node pkg="map_server" type="map_server" name="map_server" args="/$(find amcl)/amcl_data/maps/xxxxxxxx.yaml"/> 
```
3.確認好各frame的id及scan matching method是PLICP
```
<node pkg="accurate_localization" type="ICP_with_AMCL" name="ICP_with_AMCL" output="screen">
        <param name="amcl_time_save_path" type="string" value="$(find accurate_localization)/result/amcl_time.csv" />
        .
        .
        .
        <param name="odom_frame_id" value="odom_frame"/>
        <param name="base_frame_id" value="base_link"/>
        <param name="Lidar_frame_id" value="laser"/>
        <param name="global_frame_id" value="map"/>
        .
        .
        .
        <param name="scan_match_method" type="string" value="PLICP"/>
        
    </node>
```
4.看需不需要用原始/gt data轉換出base_link ground truth，可選擇要不要設置base_link初始位置校正及tracker相對車身夾角校正，會publish位置結果到/base_gt上
```
    <node pkg="accurate_localization" type="base_gt" name="base_gt" output="screen">
        <param name="do_calibration" value="false"/>
        <param name="vive_yaw_edit" value="false"/>
        <param name="base_link_initial_pose_x" value="0.019625"/>
        <param name="base_link_initial_pose_y" value="-0.005563"/>
        <param name="base_link_initial_pose_yaw" value="-0.002845"/>
        <param name="edition_plus_on_tracker_yaw" value="-0.002"/>
    </node>
```
##### 設置好後開始跑定位
step1 : 跑accurate_localization包中==my_localize.launch==

```
roslaunch accurate_localization my_localize.launch 
```
step2 : 接著跑實驗bag檔
```
rosbag play xxxxxx.bag --clock 
```

**3.計算結果**
可跑==lab_localization/bash/evo.sh==
map的.yaml放入/lab_localization/bag內
還要傳入要用的ground_truth的topic（用車上學長程式算的是base_gt_odom)
```
bash evo.sh <bag_file> <output_folder_name> <yaml_file> <ground_truth_topic>
```
結果會包在/lab_localization/evo_result/<output_folder_name>資料夾內
**4.結果轉excel**
將/lab_localization/evo_result/<output_folder_name>結果資料夾放到windows系統下，跑==evo_result_format.py==,程式內要設好輸入輸出路徑
base_path ＝ 結果資料夾路徑
output_excel_path ＝ 要輸出的excel檔路徑
