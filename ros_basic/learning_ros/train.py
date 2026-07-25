#!/usr/bin/env python3
"""
train.py
────────
Script huấn luyện mô hình PyTorch Offline dùng cho cơ chế DAgger (Imitation Learning).

Quy trình:
  1. Đọc dữ liệu từ file `dagger_dataset.csv`.
  2. Normalize dữ liệu (LiDAR đầu vào và nhãn đầu ra).
  3. Chia dữ liệu thành tập Train / Validation.
  4. Đưa vào PyTorch DataLoader để huấn luyện qua mạng MLP (Multi-Layer Perceptron).
  5. Đánh giá loss trên tập Val, lưu lại trọng số tốt nhất thành file `dagger_model.pth`.
"""

import os
import csv
import numpy as np

# PyTorch Imports
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader
except ImportError:
    raise ImportError("Thiếu thư viện PyTorch! Hãy cài đặt bằng lệnh: pip install torch torchvision")

# --- 1. Định nghĩa Mạng Neural (Đồng bộ với dagger_inference.py) ---
class DAggerMLP(nn.Module):
    def __init__(self, input_dim=60, output_dim=2):
        super(DAggerMLP, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_dim)
        )
        
    def forward(self, x):
        return self.network(x)

# --- 2. Dataset Custom cho DAgger ---
class DAggerDataset(Dataset):
    def __init__(self, csv_path, input_dim=60):
        self.inputs = []
        self.targets = []

        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"Không tìm thấy file dataset tại: {csv_path}. Hãy chạy thu thập dữ liệu trước!")

        # Đọc dữ liệu từ CSV (dùng thư viện csv thuần để tránh phụ thuộc pandas)
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader) # Bỏ qua header
            
            for row in reader:
                if len(row) < (input_dim + 2):
                    continue
                # 60 cột đầu là LiDAR
                lidar_data = [float(val) for val in row[:input_dim]]
                # 2 cột cuối là linear_v và angular_z
                cmd_data = [float(row[input_dim]), float(row[input_dim+1])]
                
                self.inputs.append(lidar_data)
                self.targets.append(cmd_data)

        self.inputs = np.array(self.inputs, dtype=np.float32)
        self.targets = np.array(self.targets, dtype=np.float32)

        # Normalize đầu vào (Min-Max Scaling cho khoảng đo của LiDAR: 0m -> 10m)
        self.inputs = self.inputs / 10.0  # Đưa về khoảng [0, 1]

        print(f"Loaded dataset from {csv_path}")
        print(f"Total samples: {len(self.inputs)}")
        print(f"Input shape: {self.inputs.shape} | Target shape: {self.targets.shape}")

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        return self.inputs[idx], self.targets[idx]


# --- 3. Tiến trình Huấn luyện ---
def train_model(csv_path, save_path, epochs=100, batch_size=64, lr=0.001, val_split=0.2):
    # Khởi tạo dataset
    dataset = DAggerDataset(csv_path)
    
    # Chia tập Train/Val
    val_size = int(len(dataset) * val_split)
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])

    # DataLoaders
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # Khởi tạo Model, Loss, Optimizer
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = DAggerMLP(input_dim=60, output_dim=2).to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    best_val_loss = float('inf')

    print(f"\nTraining on: {device}")
    print(f"Epochs: {epochs} | Batch Size: {batch_size} | Learning Rate: {lr}\n")

    for epoch in range(epochs):
        # --- Epoch Train ---
        model.train()
        train_loss = 0.0
        for inputs, targets in train_loader:
            inputs, targets = inputs.to(device), targets.to(device)
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item() * inputs.size(0)
        train_loss /= len(train_loader.dataset)

        # --- Epoch Validation ---
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                val_loss += loss.item() * inputs.size(0)
        val_loss /= len(val_loader.dataset)

        # In log định kỳ
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"Epoch [{epoch+1}/{epochs}] | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}")

        # Lưu model có Val Loss tốt nhất
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            torch.save(model.state_dict(), save_path)

    print(f"\nTraining Finished! Best Val Loss: {best_val_loss:.6f}")
    print(f"Best model weights saved to: {save_path}")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Train DAgger Model for F1TENTH")
    parser.add_argument('--csv', type=str, 
                        default=os.path.expanduser('~/ros2_ws/src/ros_basic/learning_ros/dagger_dataset.csv'),
                        help='Đường dẫn tới file CSV dataset')
    parser.add_argument('--model', type=str, 
                        default=os.path.expanduser('~/ros2_ws/src/ros_basic/learning_ros/dagger_model.pth'),
                        help='Đường dẫn lưu file trọng số mô hình (.pth)')
    parser.add_argument('--epochs', type=int, default=80, help='Số lượng epochs huấn luyện')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    
    args = parser.parse_args()
    
    train_model(
        csv_path=args.csv,
        save_path=args.model,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        val_split=0.2
    )
