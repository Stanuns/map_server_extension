# map_server_extension

## 地图管理
-- 涉及地图名称的均不需要包含地图扩展名称(pgm,yaml)
### 1 获取地图列表
-- service名称：/get_maps_name_list
-- 传入参数
无

-- 调用成功返回参数
result: True
name_list: '测试地图1;测试地图2;测试地图3'
message ''

-- 调用失败返回参数
result: False
name_list: ''
message 'Failed to open maps directory.'


### 2 获取地图(文件)
-- service名称：/get_map_file
-- 传入参数
map_name: '测试地图1'

-- 调用成功返回参数
result: True
message: 'Successfully retrieved map files: 测试地图1'
yaml_content: array('B',[105,109,230,139,.....])
pgm_content: array('B',[80,53,10,52,54,....])

-- 调用失败返回参数
result: False
message 'Failed to read YAML file: ...'
yaml_content: array('B')
pgm_content: array('B')

### 3 设置当前地图(导航时用)
-- service名称：/set_current_map
-- 传入参数
map_name: '测试地图1'

-- 调用成功返回参数
result: True
err_code: '200'
err_msg: 'Successfully set current_map_name to 测试地图1'

-- 调用失败返回参数
result: False
err_code: '500'
err_msg: 'Failed to set map_mgmt.yaml: ...'

### 4 下发地图
-- service名称：/update_map_file
-- 传入参数
map_name: '测试地图1'
yaml_content: array('B',[105,109,230,139,.....])
pgm_content: array('B',[80,53,10,52,54,....])

-- 调用成功返回参数
result: True
err_code: '200'
err_msg: 'Map file updated successfully'

-- 调用失败返回参数
result: False
err_code: '500'
err_msg: 'Failed to update map file: ...'

### 5 删除地图
-- service名称：/remove_map_file
-- 传入参数
map_name: '测试地图1'

-- 调用成功返回参数
result: True
message: 'Successfully deleted map files: 测试地图1.pgm and 测试地图1.yaml'

-- 调用失败返回参数
result: False
message: 'Failed to delete PGM file: 测试地图1.pgm'
