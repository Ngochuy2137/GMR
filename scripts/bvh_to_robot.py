import argparse
import pathlib
import time
from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.lafan1 import load_lafan1_file
from rich import print
from tqdm import tqdm
import os
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

def plot_dof_timeseries(dof_pos: np.ndarray, fps: float = 60.0, save_dir: str = "dof_plots",
                        downsample: int = 1, show: bool = False):
    """
    参数
    ----
    dof_pos : np.ndarray
        形状 (T, 30)。每列是一个 DOF 在时间上的取值。
    fps : float
        用于生成时间轴（秒）。如果没有 fps，用默认 60。
    save_dir : str
        图片输出目录。
    downsample : int
        下采样步长（>=1）。例如设置 5 可减少绘图点数（提速）。
    show : bool
        是否在绘制后弹出窗口显示（服务器/无显示环境请保持 False）。
    """
    assert dof_pos.ndim == 2 and dof_pos.shape[1] == 30, f"Expected (T,30), got {dof_pos.shape}"
    os.makedirs(save_dir, exist_ok=True)

    T = dof_pos.shape[0]
    idx = np.arange(T)[::downsample]
    t = idx / float(fps)  # 时间轴（秒）
    for j in range(30):
        y = dof_pos[idx, j]

        plt.figure()  # 每个关节独立一张图（不要用子图）
        plt.plot(t, y, linewidth=1.0)
        plt.xlabel("Time (s)")
        plt.ylabel(f"DOF {j} value")
        plt.title(f"DOF {j} Timeseries")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.legend([f"DOF_{j}"])  # 用 index 进行命名

        out_path = os.path.join(save_dir, f"dof_{j:02d}.png")
        plt.tight_layout()
        plt.savefig(out_path, dpi=150)
        if show:
            plt.show()
        plt.close()

    print(f"[OK] Saved {30} plots to: {os.path.abspath(save_dir)}")
if __name__ == "__main__":
    
    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bvh_file",
        help="BVH motion file to load.",
        required=True,
        type=str,
    )
    
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "booster_t1", "stanford_toddy", "fourier_n1", "engineai_pm01","taihu"],
        default="unitree_g1",
    )
        
    parser.add_argument(
        "--record_video",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--video_path",
        type=str,
        default="videos/example.mp4",
    )

    parser.add_argument(
        "--rate_limit",
        action="store_true",
        default=False,
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the robot motion.",
    )
    
    
    args = parser.parse_args()
    

    if args.save_path is not None:
        save_dir = os.path.dirname(args.save_path)
        if save_dir:  # Only create directory if it's not empty
            os.makedirs(save_dir, exist_ok=True)
        qpos_list = []

    
    # Load SMPLX trajectory
    lafan1_data_frames, actual_human_height, fps = load_lafan1_file(args.bvh_file)
    # import ipdb; ipdb.set_trace()
    
    # Initialize the retargeting system
    retargeter = GMR(
        src_human="bvh",
        tgt_robot=args.robot,
        actual_human_height=actual_human_height,
    )

    motion_fps = fps
    
    robot_motion_viewer = RobotMotionViewer(robot_type=args.robot,
                                            motion_fps=motion_fps,
                                            transparent_robot=0,
                                            record_video=args.record_video,
                                            video_path=args.video_path,
                                            # video_width=2080,
                                            # video_height=1170
                                            )
    
    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds
    
    print(f"mocap_frame_rate: {motion_fps}")
    
    # Create tqdm progress bar for the total number of frames
    pbar = tqdm(total=len(lafan1_data_frames), desc="Retargeting")
    
    # Start the viewer
    i = 0
    window_size = 5  # 平滑窗口大小，可以调整
    qpos_buffer = deque(maxlen=window_size)

    while i < len(lafan1_data_frames):
        
        # FPS measurement
        fps_counter += 1
        current_time = time.time()
        if current_time - fps_start_time >= fps_display_interval:
            actual_fps = fps_counter / (current_time - fps_start_time)
            print(f"Actual rendering FPS: {actual_fps:.2f}")
            fps_counter = 0
            fps_start_time = current_time
            
        # Update progress bar
        pbar.update(1)

        # Update task targets.
        smplx_data = lafan1_data_frames[i]
        # retarget
        qpos = retargeter.retarget(smplx_data)
        
        # Smooth the qpos using a moving average
        qpos_buffer.append(qpos)
        qpos_smooth = np.mean(qpos_buffer, axis=0)

        # visualize
        robot_motion_viewer.step(
            root_pos=qpos_smooth[:3],
            root_rot=qpos_smooth[3:7],
            dof_pos=qpos_smooth[7:],
            human_motion_data=retargeter.scaled_human_data,
            rate_limit=args.rate_limit,
            # human_pos_offset=np.array([0.0, 0.0, 0.0])
        )
        i += 1
       
        if args.save_path is not None:
            qpos_list.append(qpos_smooth)
    
    if args.save_path is not None:
        import pickle
        root_pos = np.array([qpos[:3] for qpos in qpos_list])
        # save from wxyz to xyzw
        root_rot = np.array([qpos[3:7][[1,2,3,0]] for qpos in qpos_list])
        dof_pos = np.array([qpos[7:] for qpos in qpos_list])
        
        motion_data = {
            "fps": motion_fps,
            "root_pos": root_pos,
            "root_rot": root_rot,
            "dof_pos": dof_pos
        }
        with open(args.save_path, "wb") as f:
            pickle.dump(motion_data, f)
        print(f"Saved to {args.save_path}")
        # plot_dof_timeseries(dof_pos, fps=motion_fps, save_dir="dof_plots", downsample=1)
    # Close progress bar
    pbar.close()
    
    robot_motion_viewer.close()
       
