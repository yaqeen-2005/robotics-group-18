"""Train and run a small CNN for door-like traversability belief."""

import argparse
import json
import random
import zipfile
from pathlib import Path

import numpy as np
import torch
from torch import nn

from common.cell_attributes import DOOR_BELIEF, TEMPORAL_DEFAULTS

ROOT = Path(__file__).parent
ARCHIVE = ROOT / "training_data" / "door_temporal_v1.zip"
MODEL = ROOT / "weights" / "door_belief.pt"


def patch(attributes, x, y, size=11):
    if size < 3 or size % 2 != 1:
        raise ValueError("patch size must be odd and at least 3")
    result = np.zeros((9, size, size), dtype=np.float32)
    for row in range(size):
        for col in range(size):
            px, py = x + col - size // 2, y + row - size // 2
            if 0 <= px < attributes.width and 0 <= py < attributes.height:
                cell = attributes.values[py][px]
                cell = cell if len(cell) == 10 else cell + TEMPORAL_DEFAULTS
                result[:, row, col] = (
                    *cell[:4], min(cell[4] / 40, 1), min(cell[5] / 40, 1),
                    cell[6], min(cell[7] / 20, 1), cell[8],
                )
    return result


class DoorCNN(nn.Module):
    def __init__(self, size=11):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Conv2d(9, 16, 3, padding=1), nn.ReLU(),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(),
            nn.Flatten(), nn.Linear(32 * size * size, 1), nn.Sigmoid(),
        )

    def forward(self, inputs):
        return self.layers(inputs).flatten()


def generate(archive=ARCHIVE, count=2400, size=11, seed=18):
    """Synthetic 11x11 scenes: static, traffic, and regular/random moving barriers."""
    rng = random.Random(seed)
    samples = np.zeros((count, 9, size, size), np.float32)
    labels = np.zeros(count, np.float32)
    map_size = max(64, size + 2)
    center = map_size // 2
    for index in range(count):
        door = index % 2 == 0
        labels[index] = float(door)
        scene = np.zeros((9, map_size, map_size), np.float32)
        scene[0] = rng.uniform(0.7, 1)
        scene[1] = rng.uniform(0, 0.15)
        scene[2] = rng.uniform(0, 0.2)
        scene[3] = rng.uniform(0, 0.1)
        length = rng.randint(1, min(size - 2, 7))
        diagonal = rng.choice((-1, 0, 1))
        regular = rng.choice((True, False))
        observations = rng.randint(20, 40)
        if door:
            dwell = rng.randint(3, 10)
            state = rng.randint(0, 1)
            states = []
            remaining = dwell
            for _ in range(observations):
                states.append(state)
                remaining -= 1
                if remaining == 0:
                    state = 1 - state
                    remaining = dwell if regular else rng.randint(3, 12)
        else:
            state = rng.randint(0, 1)
            states = [state] * observations
            if rng.random() < 0.1:
                recent = rng.randint(1, 3)
                states[-recent:] = [1 - state] * recent
        blocks = sum(states)
        switches = sum(a != b for a, b in zip(states, states[1:]))
        for offset in range(-(length // 2), length - length // 2):
            x, y = center + offset, center + offset * diagonal
            if not (0 <= x < map_size and 0 <= y < map_size):
                continue
            scene[4, y, x] = min(observations / 40, 1)
            scene[5, y, x] = min(blocks / 40, 1)
            scene[6, y, x] = blocks / observations
            scene[7, y, x] = min(switches / 20, 1)
            scene[8, y, x] = states[-1]
        # A few irrelevant observations prevent the network from relying on a pristine background.
        for _ in range(rng.randint(0, 5)):
            x = center + rng.randrange(size) - size // 2
            y = center + rng.randrange(size) - size // 2
            if (x, y) != (center, center):
                scene[4, y, x] = rng.uniform(0.1, 1)
                scene[6, y, x] = rng.uniform(0, 1)
                scene[7, y, x] = rng.uniform(0, 0.15)
        half = size // 2
        samples[index] = scene[:, center - half:center + half + 1,
                               center - half:center + half + 1]
    archive = Path(archive)
    archive.parent.mkdir(parents=True, exist_ok=True)
    import io
    buffer = io.BytesIO()
    np.savez_compressed(buffer, samples=samples, labels=labels)
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as file:
        file.writestr("door_samples.npz", buffer.getvalue())
        file.writestr("metadata.json", json.dumps({"seed": seed, "count": count, "patch_size": size, "map_size": map_size, "sample_interval_seconds": 0.5, "synthetic": True}))
    return archive


def train(archive=ARCHIVE, output=MODEL, epochs=8, seed=18):
    torch.manual_seed(seed)
    with zipfile.ZipFile(archive) as file:
        import io
        data = np.load(io.BytesIO(file.read("door_samples.npz")))
        inputs = torch.from_numpy(data["samples"])
        labels = torch.from_numpy(data["labels"])
    size = inputs.shape[-1]
    model = DoorCNN(size)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    split = int(len(inputs) * 0.8)
    for _ in range(epochs):
        model.train()
        for start in range(0, split, 64):
            output_values = model(inputs[start:start + 64])
            loss = nn.functional.binary_cross_entropy(output_values, labels[start:start + 64])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    model.eval()
    with torch.no_grad():
        predictions = model(inputs[split:])
        accuracy = ((predictions >= 0.5) == labels[split:].bool()).float().mean().item()
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"state_dict": model.state_dict(), "size": size, "validation_accuracy": accuracy}, output)
    return accuracy


class DoorPredictor:
    def __init__(self, model_path=MODEL):
        checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
        self.size = checkpoint["size"]
        self.model = DoorCNN(self.size)
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.eval()

    def predict(self, attributes, x, y):
        with torch.no_grad():
            return float(self.model(torch.from_numpy(patch(attributes, x, y, self.size))[None])[0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=2400)
    parser.add_argument("--size", type=int, default=11)
    parser.add_argument("--epochs", type=int, default=8)
    args = parser.parse_args()
    generate(count=args.samples, size=args.size)
    print(f"held-out synthetic accuracy: {train(epochs=args.epochs):.3f}")
