import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paketlerin yerlerini buluyoruz
    pkg_gas_sim = get_package_share_directory('gas_distribution_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Kullanıcının ev dizinini buluyoruz (/home/sefa)
    home_dir = os.environ['HOME']

    # 1. DARPA Mağara Modellerinin Yeri
    darpa_models_path = os.path.join(home_dir, 'araswarm_ws', 'src', 'gas_distribution_sim', 'models', 'subt_cave_sim', 'models')
    
    # 2. PX4 Pluginlerinin (Beyin Bağlantısı) Yeri - KRİTİK KISIM
    # Bu klasörde 'libgazebo_mavlink_interface.so' dosyası duruyor.
    px4_build_path = os.path.join(home_dir, 'PX4-Autopilot', 'build', 'px4_sitl_default', 'build_gazebo-classic')

    # Ortam değişkenlerini (Environment Variables) hazırlıyoruz
    
    # MODEL YOLU (GAZEBO_MODEL_PATH)
    # Var olan yolları koruyup, üzerine bizimkileri ekliyoruz
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = darpa_models_path + os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    else:
        model_path = darpa_models_path

    # PLUGIN YOLU (GAZEBO_PLUGIN_PATH)
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        plugin_path = px4_build_path + os.pathsep + os.environ['GAZEBO_PLUGIN_PATH']
    else:
        plugin_path = px4_build_path

    # KÜTÜPHANE YOLU (LD_LIBRARY_PATH)
    if 'LD_LIBRARY_PATH' in os.environ:
        ld_path = px4_build_path + os.pathsep + os.environ['LD_LIBRARY_PATH']
    else:
        ld_path = px4_build_path

    return LaunchDescription([
        # Ayarladığımız yolları sisteme yüklüyoruz
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld_path),
        
        # Gazebo'yu ve Mağarayı Başlatıyoruz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            # verbose=true yaptık ki hata verirse görelim
            launch_arguments={'world': os.path.join(pkg_gas_sim, 'worlds', 'cave.world'), 'verbose': 'true'}.items(),
        ),
    ])