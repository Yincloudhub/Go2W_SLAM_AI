# Robot-Side Files

This directory contains files that are meant to be version-controlled for the GO2W robot deployment.

It intentionally does not contain:

- GGUF model weights.
- llama.cpp build outputs.
- runtime logs.
- temporary inference outputs.
- robot-generated maps or point clouds.

## Runtime Layout On Robot

The git clone lives at:

```text
/home/unitree/Go2W_SLAM_AI
```

Runtime files are installed outside the repo:

```text
/home/unitree/llm_runtime
/home/unitree/models
/home/unitree/slam_gateway_refactor
```

This keeps the PC-side repository, robot-side git clone, model storage, and executable runtime separated.

## Managed Files

```text
robot/
  README.md
  llm_runtime/
    ask_qwen.sh
    install_runtime_files.sh
```

`ask_qwen.sh` is versioned here, then installed to:

```text
/home/unitree/llm_runtime/scripts/ask_qwen.sh
```

## External Files

The model is external to git:

```text
/home/unitree/models/Qwen_Qwen3-4B-Instruct-2507-Q4_K_M.gguf
```

Expected SHA256:

```text
2fde00ce69dd4899c70d020845e2638353015bba0fdf161b3eb965f2bca4464e
```

