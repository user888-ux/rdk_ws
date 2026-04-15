#!/usr/bin/env python3
"""
修复URDF中的mesh路径
"""
import os
import re

def fix_urdf_paths(input_file, output_file):
    print(f"修复URDF路径: {input_file}")
    
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 修复1: 将model://改为package://
    fixed_content = re.sub(
        r'filename="model://my_drone_description/meshes/(.+?)"',
        r'filename="package://my_drone_description/meshes/\1"',
        content
    )
    
    # 修复2: 去掉scale="100 100 100"（这是主要问题！）
    fixed_content = re.sub(
        r'scale="100 100 100"',
        r'scale="1 1 1"',
        fixed_content
    )
    
    # 修复3: 确保XML声明正确
    if not fixed_content.startswith('<?xml'):
        fixed_content = '<?xml version="1.0"?>\n' + fixed_content
    
    # 修复4: 去掉不合理的joint限制
    fixed_content = fixed_content.replace('lower="0"', 'lower="-3.14"')
    fixed_content = fixed_content.replace('upper="3.14"', 'upper="3.14"')
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(fixed_content)
    
    print(f"修复后的URDF已保存到: {output_file}")
    
    # 比较修改
    original_lines = content.count('\n')
    fixed_lines = fixed_content.count('\n')
    print(f"原始文件: {original_lines} 行")
    print(f"修复文件: {fixed_lines} 行")
    
    return output_file

def create_simple_test_urdf():
    """创建一个简单的测试URDF"""
    simple_urdf = """<?xml version="1.0"?>
<robot name="simple_drone">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
  <link name="link0">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  <joint name="joint0" type="revolute">
    <parent link="base_link"/>
    <child link="link0"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  </joint>
</robot>"""
    
    test_path = "/tmp/simple_drone.urdf"
    with open(test_path, 'w') as f:
        f.write(simple_urdf)
    
    print(f"简单测试URDF已创建: {test_path}")
    return test_path

def main():
    # 原始URDF路径
    original_urdf = "/home/user-888/dh_robot_ws/install/my_drone_description/share/my_drone_description/urdf/drone.urdf"
    
    # 如果安装目录没有，检查src目录
    if not os.path.exists(original_urdf):
        original_urdf = "/home/user-888/dh_robot_ws/src/my_drone_description/urdf/drone.urdf"
    
    if not os.path.exists(original_urdf):
        print("错误: 未找到URDF文件!")
        # 创建测试URDF
        test_urdf = create_simple_test_urdf()
        print(f"请使用测试URDF: {test_urdf}")
        return test_urdf
    
    # 修复URDF
    fixed_urdf = original_urdf + ".fixed"
    fix_urdf_paths(original_urdf, fixed_urdf)
    
    print("\n=== 下一步 ===")
    print(f"1. 检查修复后的URDF: nano {fixed_urdf}")
    print(f"2. 备份原文件: cp {original_urdf} {original_urdf}.backup")
    print(f"3. 使用修复后的URDF: cp {fixed_urdf} {original_urdf}")
    print(f"4. 重新构建: cd ~/dh_robot_ws && colcon build --packages-select my_drone_description")
    print(f"5. 测试: source install/setup.bash && ros2 launch my_drone_description display.launch.py")
    
    return fixed_urdf

if __name__ == "__main__":
    main()