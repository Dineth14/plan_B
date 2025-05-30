import os
import shutil
from pathlib import Path

def copy_source_files():
    # Get the current directory (where this script is located)
    current_dir = Path(__file__).parent
    
    # Get the source directory (two levels up from current directory)
    source_dir = Path(__file__).parent.parent.parent.parent / 'two_wheel_wall_following' / 'src'
    
    # Create necessary directories if they don't exist
    for dir_name in ['adapters', 'core', 'filters', 'motor_control', 'config']:
        os.makedirs(current_dir / dir_name, exist_ok=True)
    
    # Copy all Python files from source directories
    for dir_name in ['adapters', 'core', 'filters', 'motor_control', 'config']:
        src_dir = source_dir / dir_name
        dst_dir = current_dir / dir_name
        
        if src_dir.exists():
            for file in src_dir.glob('*.py'):
                shutil.copy2(file, dst_dir)
                print(f'Copied {file} to {dst_dir}')

if __name__ == '__main__':
    copy_source_files()
    print('Source files copied successfully!') 