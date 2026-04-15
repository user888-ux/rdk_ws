#!/bin/bash
# 修复URDF中的mesh路径

URDF_FILE="/home/user-888/dh_robot_ws/src/my_drone_description/urdf/drone0.urdf"
BACKUP_FILE="${URDF_FILE}.backup"
MESHES_PATH="/home/user-888/dh_robot_ws/src/my_drone_description/meshes"

echo "备份原始URDF: $BACKUP_FILE"
cp "$URDF_FILE" "$BACKUP_FILE"

echo "将package://替换为file://绝对路径"
sed -i "s|package://my_drone_description/meshes/|file://${MESHES_PATH}/|g" "$URDF_FILE"

echo "修复完成！"
echo "原始文件备份在: $BACKUP_FILE"
echo "新文件: $URDF_FILE"
