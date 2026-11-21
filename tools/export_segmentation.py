import argparse
import torch
import segmentation_models_pytorch as smp


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--weights", required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    saved = torch.load(args.weights, map_location="cpu", weights_only=True)
    model = smp.Unet(encoder_name=saved["encoder"], encoder_weights=None, classes=saved["classes"])
    model.load_state_dict(saved["state_dict"]); model.eval()
    torch.onnx.export(model, torch.zeros(1, 3, 384, 640), args.output, input_names=["image"], output_names=["logits"], opset_version=17)
    print(args.output)


if __name__ == "__main__":
    main()
