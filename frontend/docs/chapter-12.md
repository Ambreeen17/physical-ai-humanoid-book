---
sidebar_position: 13
title: "Chapter 12: Vision-Language-Action Models"
---

# Chapter 12: Vision-Language-Action (VLA) Models

<PersonalizationToggle chapterId="12" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Understand VLA architectures** (RT-1, RT-2, Octo)
2. **Tokenize robot actions** for transformer models
3. **Fine-tune VLMs** for robotic control
4. **Implement semantic instruction following**
5. **Evaluate generalization** to new objects and instructions

---

## Introduction

In Chapter 5, we used vision to see objects. In Chapter 11, we used imitation to learn tasks. But how do we tell a robot: "Pick up the extinct animal toy" without explicitly programming an "extinct animal" detector?

**Vision-Language-Action (VLA)** models combine the reasoning of Large Language Models (LLMs) with robotic control, enabling semantic understanding of the physical world.

---

## Section 12.1: The VLA Architecture

A VLA model takes **Images + Text** as input and outputs **Robot Actions**.

### Tokenizing Actions

Language models output tokens (text). To make them control robots, we must discretize continuous actions into tokens.

**Action Space:**
- $x, y, z$ (Cartesian position)
- $rx, ry, rz$ (Rotation)
- $gripper$ (Open/Close)

We quantize each dimension into 256 bins (tokens).

```python
class ActionTokenizer:
    """
    Convert continuous actions to discrete tokens and back.
    """
    def __init__(self, action_min, action_max, num_bins=256):
        self.min = np.array(action_min)
        self.max = np.array(action_max)
        self.bins = num_bins

    def tokenize(self, action):
        """Continuous action -> discrete tokens."""
        # Normalize to [0, 1]
        norm = (action - self.min) / (self.max - self.min)
        norm = np.clip(norm, 0, 1)

        # Quantize
        tokens = (norm * (self.bins - 1)).astype(int)
        return tokens

    def detokenize(self, tokens):
        """Discrete tokens -> continuous action."""
        # De-quantize
        norm = tokens / (self.bins - 1)

        # Denormalize
        action = norm * (self.max - self.min) + self.min
        return action
```

---

## Section 12.2: RT-1 (Robotics Transformer)

RT-1 uses an EfficientNet to process images and a Transformer to attend to instruction history and image tokens.

```python
import torch
import torch.nn as nn

class RT1Model(nn.Module):
    def __init__(self, vocab_size=1000, action_bins=256):
        super().__init__()
        # 1. Vision Encoder (EfficientNet-B3)
        self.vision_encoder = EfficientNet.from_pretrained('efficientnet-b3')

        # 2. Language Encoder (Universal Sentence Encoder)
        self.lang_encoder = UniversalSentenceEncoder()

        # 3. FiLM Layers (Fuse Vision + Language)
        self.film_fusion = FiLMLayer()

        # 4. Token Learner (Reduce image tokens)
        self.token_learner = TokenLearner(num_tokens=8)

        # 5. Transformer Decoder
        self.transformer = nn.TransformerDecoder(
            nn.TransformerDecoderLayer(d_model=512, nhead=8),
            num_layers=6
        )

        # 6. Action Head
        self.action_head = nn.Linear(512, 7 * action_bins) # 7 dims * 256 bins

    def forward(self, image, instruction):
        # Encode image
        img_features = self.vision_encoder(image)

        # Encode instruction
        text_embedding = self.lang_encoder(instruction)

        # Fuse
        fused_features = self.film_fusion(img_features, text_embedding)

        # Token learner
        visual_tokens = self.token_learner(fused_features)

        # Transformer
        output = self.transformer(visual_tokens, text_embedding)

        # Predict action logits
        logits = self.action_head(output)
        return logits.view(-1, 7, 256)
```

---

## Section 12.3: RT-2 (Vision-Language-Action)

RT-2 takes a pre-trained VLM (like PaLI-X or PaLM-E) and fine-tunes it to output action tokens as text.

Example Prompt:
`"Instruction: Pick up the apple. Image: [IMAGE] Action:"`

Example Output:
`"128 54 200 12 10 0 255"` (representing x,y,z,rx,ry,rz,gripper)

### Fine-Tuning a VLM

```python
from transformers import AutoModelForCausalLM, AutoProcessor

class RT2Agent:
    def __init__(self):
        self.model = AutoModelForCausalLM.from_pretrained("google/paligemma-3b-pt-224")
        self.processor = AutoProcessor.from_pretrained("google/paligemma-3b-pt-224")

    def predict_action(self, image, text_instruction):
        prompt = f"Instruction: {text_instruction} Action: "
        inputs = self.processor(text=prompt, images=image, return_tensors="pt")

        # Generate output tokens
        generated_ids = self.model.generate(**inputs, max_new_tokens=7)

        # Decode tokens to numbers
        action_str = self.processor.batch_decode(generated_ids)[0]
        # Parse "128 54..." string into numpy array
        actions = self.parse_action_string(action_str)
        return actions

    def parse_action_string(self, s):
        # Extract numbers from string
        # ...
        return np.array([float(x) for x in s.split()])
```

---

## Section 12.4: Octo (Open Source VLA)

Octo is an open-source model trained on the Open X-Embodiment dataset (data from 20+ robot types). It uses a specialized Transformer architecture designed for efficient inference.

### Using Octo for Inference

```python
import octo
from octo.model.octo_model import OctoModel

# Load pretrained model
model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")

# Run inference
def run_octo_policy(model, image, instruction):
    # Prepare observation
    observation = {
        "image_primary": image,
        "language_instruction": instruction
    }

    # Sample action
    actions = model.sample_actions(
        observation,
        unnormalization_statistics=model.dataset_statistics["action"],
        rng=jax.random.PRNGKey(0)
    )

    return actions[0] # Next action
```

---

## Section 12.5: Reasoning & Chain of Thought

VLAs can do more than just follow commands; they can **reason**.

User: "I'm hungry."
Robot (Thinking):
1. User is hungry -> needs food.
2. I see an apple and a sponge.
3. Apple is food.
4. Plan: Pick up apple.
Action: [Pick up apple]

This emerging capability allows robots to handle ambiguous instructions and perform multi-step semantic reasoning.

---

## Summary

### Key Takeaways

1. **VLAs** unify perception, reasoning, and control into a single model.
2. **Action Tokenization** allows using standard LLM architectures for control.
3. **RT-1** learns from scratch; **RT-2** leverages pre-trained VLMs.
4. **Generalization** is the main benefit: VLAs handle unseen objects and rewording.
5. **Inference Latency** is the main challenge: running a 3B+ param model at 10Hz is hard.

### Looking Ahead

In Chapter 13, we'll put everything together into a complete **System Integration**, building a fully functional robot software stack.

---

**Chapter completed**: 2026-01-20
**Chapter**: 12 - Vision-Language-Action Models
