"""Four-class paint/stop-line/road segmentation, split strictly by recording run."""
import argparse
import json
from pathlib import Path
import random
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import segmentation_models_pytorch as smp


class Frames(Dataset):
    def __init__(self, rows):
        self.rows = rows

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        image = cv2.imread(row["image"])
        mask = cv2.imread(row["mask"], cv2.IMREAD_UNCHANGED)
        if image is None or mask is None:
            raise ValueError(f"image/mask missing: {row}")
        if mask.ndim != 2 or not np.isin(mask, [0, 1, 2, 3]).all() or mask.shape != image.shape[:2]:
            raise ValueError("mask must be original-resolution 1-channel class IDs 0..3, not a coloured preview")
        image = cv2.cvtColor(cv2.resize(image, (640, 384)), cv2.COLOR_BGR2RGB).astype(np.float32)/255.
        image = (image-np.array([.485, .456, .406], np.float32))/np.array([.229, .224, .225], np.float32)
        mask = cv2.resize(mask, (640, 384), interpolation=cv2.INTER_NEAREST).astype(np.int64)
        return torch.from_numpy(image.transpose(2, 0, 1)), torch.from_numpy(mask)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--manifest", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch", type=int, default=8)
    p.add_argument("--pretrained", action="store_true", help="download ImageNet encoder weights if uncached")
    args = p.parse_args()
    random.seed(7); np.random.seed(7); torch.manual_seed(7)
    rows = [json.loads(line) for line in Path(args.manifest).read_text().splitlines()]
    splits = {name: [r for r in rows if r["split"] == name] for name in ("train", "val", "test")}
    runs = {name: {r["run_id"] for r in group} for name, group in splits.items()}
    if any(runs[a] & runs[b] for a, b in (("train", "val"), ("train", "test"), ("val", "test"))):
        raise ValueError("recording run leakage between splits")
    if not splits["train"] or not splits["val"]:
        raise ValueError("need separately recorded labelled train and validation sets")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = smp.Unet(encoder_name="mobilenet_v2", encoder_weights="imagenet" if args.pretrained else None, classes=4).to(device)
    loaders = {name: DataLoader(Frames(splits[name]), batch_size=args.batch, shuffle=name == "train", num_workers=0) for name in ("train", "val")}
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-4)
    ce = torch.nn.CrossEntropyLoss(weight=torch.tensor([1., 4., 5., 1.], device=device))
    dice = smp.losses.DiceLoss(mode="multiclass", from_logits=True)
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    (output/"data_manifest.jsonl").write_text(Path(args.manifest).read_text())
    best = float("inf")
    history = []
    for epoch in range(args.epochs):
        metrics = {"epoch": epoch+1}
        for split in ("train", "val"):
            model.train(split == "train")
            total = 0.
            confusion = torch.zeros((4, 4), device=device, dtype=torch.int64)
            with torch.set_grad_enabled(split == "train"):
                for images, masks in loaders[split]:
                    images, masks = images.to(device), masks.to(device)
                    logits = model(images)
                    loss = ce(logits, masks)+dice(logits, masks)
                    if split == "train":
                        optimizer.zero_grad(set_to_none=True); loss.backward(); optimizer.step()
                    total += loss.item()*len(images)
                    indices = masks.flatten()*4+logits.argmax(1).flatten()
                    confusion += torch.bincount(indices, minlength=16).reshape(4, 4)
            loss_value = total/len(splits[split])
            union = confusion.sum(0)+confusion.sum(1)-confusion.diag()
            iou = (confusion.diag()/union.clamp(min=1)).cpu().tolist()
            metrics[split] = {"loss": loss_value, "iou_by_class": iou}
            if split == "val" and loss_value < best:
                best = loss_value
                torch.save({"state_dict": model.state_dict(), "encoder": "mobilenet_v2", "classes": 4, "size": [640, 384]}, output/"best.pt")
        history.append(metrics)
        (output/"history.json").write_text(json.dumps(history, indent=2))
        print(json.dumps(metrics), flush=True)


if __name__ == "__main__":
    main()
