# 地图配置文件
# 在这里设置当前使用的地图名称

# 当前激活的地图 (不包含扩展名)
CURRENT_MAP=hospital_0.1

# 可用的地图列表
AVAILABLE_MAPS=(
    "kitchen_5"
    "hospital_0.1"
    "lobby_1"
    "lobby_5"
    "restaurant_5"
    "isaacwarehouse_5"
    # 在这里添加更多地图
)

# 地图文件路径
MAP_DIR="/home/getting/Sweeping-Robot/src/auto_nav/map"

# 验证地图文件是否存在
validate_map() {
    local map_name=$1
    if [[ ! -f "${MAP_DIR}/${map_name}.yaml" ]]; then
        echo "错误: 地图文件 ${MAP_DIR}/${map_name}.yaml 不存在!"
        return 1
    fi
    if [[ ! -f "${MAP_DIR}/${map_name}.pgm" ]]; then
        echo "错误: 地图文件 ${MAP_DIR}/${map_name}.pgm 不存在!"
        return 1
    fi
    return 0
}
