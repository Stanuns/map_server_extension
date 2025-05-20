# map_server_extension

- 启动地图管理service server节点

```bashrc
ros2 launch map_server_extension map_mgmt_server.launch.py
```

## 地图管理

- 涉及地图名称的均不需要包含地图扩展名称(pgm,yaml)

### 1 获取地图列表

- service名称：/map_server
- 传入参数
  cmd_name: 1
  map_name: ''
  map_data: ''
- 调用成功返回参数
  err_code: 200
  err_msg: 'Successfully get maps name list
  map_list: ['测试地图1', '测试地图2',...]
  map_data: ''
- 调用失败返回参数
  err_code: 500
  err_msg: 'Failed to ...'
  map_list: []
  map_data: ’’

### 2 获取地图(文件)

- service名称：/map_server
- 传入参数
  cmd_name: 2
  map_name: '测试地图1'
  map_data: ''
- 调用成功返回参数
  err_code: 200
  err_msg: 'Successfully retrieved map: 测试地图1'
  map_list: []
  map_data: DATA
- 调用失败返回参数
  err_code: 500
  err_msg: 'error...'
  map_list: []
  map_data: ''

### 3 设置当前地图(导航时用)

- service名称：/map_server
- 传入参数
  cmd_name: 3
  map_name: '测试地图1'
  map_data: ''
- 调用成功返回参数
  err_code: 200
  err_msg: 'Successfully set current_map_name to 测试地图1'
  map_list: []
  map_data: ''
- 调用失败返回参数
  err_code: 500
  err_msg: 'Failed...'
  map_list: []
  map_data: ''

### 4 下发地图

- service名称：/map_server
- 传入参数
  cmd_name: 4
  map_name: '测试地图1'
  map_data: DATA
- 调用成功返回参数
  err_code: 200
  err_msg: 'Map file updated successfully'
  map_list: []
  map_data: ''
- 调用失败返回参数
  err_code: 500
  err_msg: 'Failed to update map file: ....'
  map_list: []
  map_data: ''
- 调用失败返回参数
  result: False
  err_code: 500
  err_msg: 'Failed to update map file: ...'

### 5 删除地图

- service名称：/map_server
- 传入参数
  cmd_name: 5
  map_name: '测试地图1'
  map_data: ''
- 调用成功返回参数
  err_code: 200
  err_msg: 'Successfully deleted map files: ...'
  map_list: []
  map_data: ''
- 调用失败返回参数
  err_code: 500
  err_msg: 'Failed to ...'
  map_list: []
  map_data: ''

### 6 保存地图

调用service名称: /save_map

- 传入参数：
  map_name: '测试地图1'

服务调用成功返回参数：
result: True
message: 'Successfully saved map, map name: 测试地图1'

服务调用失败返回参数：
result: False
message: 'Failed to save map'
