from RL.train import main
# Set action repeat to 2 instead of 8 to faster debug
# Add `--only_cpu` to faster debug
# Set replay buffer to 10
# domain_name = 'cartpole'
# domain_name = 'walker'
#
# if domain_name == 'cartpole':
#     task_name = "swingup"
#     action_repeat = 8
# else:
#     task_name = "walk"
#     action_repeat = 2
# args = f"--domain_name {domain_name} --task_name {task_name} --encoder_type pixel --action_repeat {action_repeat} --save_tb " \
#        f"--pre_transform_image_size 100 --image_size 84 --work_dir ./tmp/{domain_name} --agent sac_curl --frame_stack 3 " \
#        "--seed 1 --critic_lr 1e-3 --actor_lr 1e-3 --eval_freq 5000 --batch_size 256 --num_train_steps 1000000 " \
#        "--replay_buffer_capacity 100000 --save_model --save_buffer --save_video --load tmp/cartpole_06-08-2021-18-27-43-freeze-encoder --freeze_encoder 50000"#--only_cpu"
args = "--domain_name RealArm-v0 --cameras 0 1 --frame_stack 1 --observation_type pixel --encoder_type pixel "\
	"--save_tb --save_buffer --save_video --save_sac --work_dir real_robot_data/v0 "\
	"--eval_freq 150 --num_eval_episodes 1 --log_interval 1 "\
	"--pre_transform_image_size 100 --image_size 84 --agent rad_sac --data_aug crop "\
	"--seed 15 --batch_size 128 --num_updates 10 --num_train_steps 10000 --init_steps 0 "\
	"--reward_type v0 --replay_buffer_load_dir real_robot_demo/v0 "\
	"--synch_update --log_networks_freq 30000 --warmup_cpc 1600 --warmup_cpc_ema "\
	"--actor_log_std_max 0"

main(args.split(" "))
