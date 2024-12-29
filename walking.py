import pybullet as p
import pybullet_data
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

# === SETUP ENVIRONMENT === #
class QuadrupedEnv:
    def __init__(self):
        self.client = p.connect(p.GUI)  # Use GUI for visualization; switch to p.DIRECT for headless
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("laikago/laikago.urdf", [0, 0, 0.5], useFixedBase=False)
        
        self.joint_ids = [i for i in range(p.getNumJoints(self.robot))]
        self.num_joints = len(self.joint_ids)
        self.state_dim = self.num_joints * 2 + 6  # Positions, velocities, and base state
        self.action_dim = self.num_joints  # Joint torques
        self.time_step = 1 / 240.0

        self.reset()

    def reset(self):
        p.resetBasePositionAndOrientation(self.robot, [0, 0, 0.5], [0, 0, 0, 1])
        for joint in self.joint_ids:
            p.resetJointState(self.robot, joint, targetValue=0, targetVelocity=0)
        self.sim_time = 0
        return self._get_observation()

    def step(self, action):
        for joint, torque in zip(self.joint_ids, action):
            p.setJointMotorControl2(self.robot, joint, p.TORQUE_CONTROL, force=torque)
        p.stepSimulation()
        self.sim_time += self.time_step
        
        obs = self._get_observation()
        reward, done = self._compute_reward_and_done(obs)
        return obs, reward, done, {}

    def _get_observation(self):
        base_pos, base_ori = p.getBasePositionAndOrientation(self.robot)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot)
        joint_states = p.getJointStates(self.robot, self.joint_ids)
        joint_positions = [s[0] for s in joint_states]
        joint_velocities = [s[1] for s in joint_states]
        obs = np.concatenate([joint_positions, joint_velocities, base_pos, base_vel, base_ang_vel])
        return obs

    def _compute_reward_and_done(self, obs):
        base_pos = obs[-6:-3]
        roll, pitch, _ = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.robot)[1])
        
        # Compute rewards
        speed_penalty = np.linalg.norm(self.target_velocity - obs[-3:])  # Match joystick velocities
        stability_penalty = 0.1 * abs(roll) + 0.1 * abs(pitch)
        reward = -speed_penalty - stability_penalty
        
        done = base_pos[2] < 0.2  # If the robot falls
        return reward, done

    def render(self):
        pass

# === DEFINE POLICY NETWORKS === #
class PPO_Policy(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.mean = nn.Linear(128, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        mean = self.mean(x)
        std = torch.exp(self.log_std)
        return mean, std

class PPO_Value(nn.Module):
    def __init__(self, state_dim):
        super().__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.value = nn.Linear(128, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return self.value(x)

# === PPO TRAINING === #
class PPO:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, epsilon=0.2):
        self.policy = PPO_Policy(state_dim, action_dim)
        self.value = PPO_Value(state_dim)
        self.policy_optim = optim.Adam(self.policy.parameters(), lr=lr)
        self.value_optim = optim.Adam(self.value.parameters(), lr=lr)
        self.gamma = gamma
        self.epsilon = epsilon

    def update(self, states, actions, log_probs, rewards, values, advantages):
        # Policy loss
        new_means, new_stds = self.policy(states)
        new_dist = Normal(new_means, new_stds)
        new_log_probs = new_dist.log_prob(actions).sum(dim=1)
        ratio = torch.exp(new_log_probs - log_probs)

        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1 - self.epsilon, 1 + self.epsilon) * advantages
        policy_loss = -torch.min(surr1, surr2).mean()

        # Value loss
        value_loss = ((rewards - values) ** 2).mean()

        # Backpropagation
        self.policy_optim.zero_grad()
        self.value_optim.zero_grad()
        (policy_loss + value_loss).backward()
        self.policy_optim.step()
        self.value_optim.step()

    def act(self, state):
        mean, std = self.policy(state)
        dist = Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=1)
        return action, log_prob

# === TRAINING LOOP === #
def train_ppo(env, ppo, num_episodes=1000, max_timesteps=200):
    for episode in range(num_episodes):
        state = env.reset()
        states, actions, rewards, log_probs, values = [], [], [], [], []
        episode_reward = 0

        for t in range(max_timesteps):
            state_tensor = torch.tensor(state, dtype=torch.float32)
            
            # Get action and log_prob from the policy network
            action, log_prob = ppo.act(state_tensor)
            value = ppo.value(state_tensor)

            # Execute action in the environment
            next_state, reward, done, _ = env.step(action.numpy())
            
            # Store the experience
            states.append(state_tensor)
            actions.append(action)
            log_probs.append(log_prob)
            rewards.append(reward)
            values.append(value)

            # Move to next state
            state = next_state
            episode_reward += reward
            
            if done:
                break
        
        # After collecting all experiences, compute advantages and update the policy
        next_state_tensor = torch.tensor(state, dtype=torch.float32)
        next_value = ppo.value(next_state_tensor)
        
        # Compute the advantages for the batch
        advantages = compute_advantages(states, rewards, next_value, dones=[done] * len(rewards))
        
        # Update the PPO policy and value networks
        ppo.update(torch.stack(states), torch.stack(actions), torch.stack(log_probs), 
                   torch.tensor(rewards, dtype=torch.float32), torch.stack(values), advantages)
        
        print(f"Episode {episode+1}, Reward: {episode_reward}")


# Initialize environment and train
env = QuadrupedEnv()
ppo = PPO(env.state_dim, env.action_dim)
train_ppo(env, ppo)
