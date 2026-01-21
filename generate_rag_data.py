import json
import uuid
from datetime import datetime
import os

# --- Configurations ---
CHAPTER_NUMBER = 1
CHAPTER_TITLE = "Introduction to Physical AI"
BASE_PATH = "C:/boook"
OUTPUT_DIR = os.path.join(BASE_PATH, "specs/1-book-curriculum/chapters/chapter-1")

# Mocking token estimate (4 chars per token roughly)
def estimate_tokens(text):
    return len(text) // 4

def generate_chunk_id(section, index):
    return f"ch{CHAPTER_NUMBER}_{section}_{index:03d}"

chunks = []

# --- 1. Chapter Draft (English & Urdu combined) ---
# We'll split by main sections (headers)
chapter_content = """# Chapter 1: Introduction to Physical AI (جسمانی ذہانت کا تعارف)

## Introduction: The "Glass Wall" of AI
### (تعارف: AI کی "شفاف دیوار")

For the last decade, AI development has largely happened behind a "glass wall." Whether you were building a recommendation engine or training a Large Language Model (LLM), your code's output was digital. It lived on screens, in databases, and across fiber-optic cables. This is "Disembodied AI"—brilliant logic trapped in a box without hands or feet.

پچھلی دہائی میں، AI کی ترقی بڑی حد تک ایک "شفاف دیوار" کے پیچھے ہوئی ہے۔ چاہے آپ ایک سفارش کا انجن بنا رہے ہوں یا ایک بڑے Language Model (LLM) کو تربیت دے رہے ہوں، آپ کے کوڈ کی پیداوار ڈیجیٹل تھی۔ یہ اسکرینوں، ڈیٹا بیسز اور فائبر آپٹک کیبلز تک محدود تھی۔ یہ "بے جسم AI" (Disembodied AI) ہے—ایک ایسی شاندار منطق جو بغیر ہاتھ یا پاؤں کے ایک ڈبے میں قید ہے۔

## 1.1 What is Embodied Intelligence?
### (1.1 جسمانی ذہانت کیا ہے؟)

Embodied Intelligence [جسمانی ذہانت] (Jasmaani Zehanat) refers to AI that is grounded in a physical form. Unlike a chatbot that only processes text, an embodied agent must navigate the messy, unpredictable physical world. It requires a **Sensorimotor Loop** [حس و حرکت کا چکر]: sensors (cameras, LiDAR) perceive the environment, the AI makes a decision, and actuators (motors, joints) execute the movement.

جسمانی ذہانت (Embodied Intelligence) سے مراد ایسی AI ہے جو ایک مادی یا جسمانی شکل میں موجود ہو۔ ایک چیٹ بوٹ کے برعکس جو صرف متن (text) پر کارروائی کرتا ہے، ایک "جسمانی ایجنٹ" کو الجھی ہوئی اور غیر متوقع مادی دنیا میں راستہ بنانا ہوتا ہے۔ اس کے لیے ایک **حس و حرکت کے چکر** (Sensorimotor Loop) کی ضرورت ہوتی ہے: حس کن (sensors) جیسے کیمرے اور لائڈار (LiDAR) ماحول کا ادراک کرتے ہیں، AI فیصلہ کرتا ہے، اور حرکت دہندہ (actuators) جیسے کہ موٹرز اور جوڑ اس حرکت کو عملی جامہ پہناتے ہیں۔

## 1.2 The Simulation-to-Real Gap
### (1.2 نقالی سے حقیقی فاصلہ)

One of the greatest challenges in robotics is the **Simulation-to-Real Gap** [نقالی سے حقیقی فاصلہ] (Naqali se Haqiqi Faasla). We often train robots in simulators like Gazebo because it is faster and safer. However, friction, lighting, and gravity in the real world are never exactly as they appear in code. Bridging this gap is the "Holy Grail" of modern Physical AI.

روبوٹکس میں سب سے بڑے چیلنجوں میں سے ایک **نقالی سے حقیقی فاصلہ** (Simulation-to-Real Gap) ہے۔ ہم اکثر روبوٹس کو Gazebo جیسے سیمولیٹرز میں تربیت دیتے ہیں کیونکہ یہ تیز اور محفوظ ہوتا ہے۔ تاہم، حقیقی دنیا میں رگڑ (friction)، روشنی اور کشش ثقل کبھی بھی ویسے نہیں ہوتے جیسے وہ کوڈ میں نظر آتے ہیں۔ اس فاصلے کو ختم کرنا جدید طبیعی AI کا ایک اہم اہم مقصد ہے۔

## 1.3 Current Humanoid Platforms (2025)
### (1.3 موجودہ انسان نما پلیٹ فارمز - 2025)

As of 2025, several humanoid platforms have emerged as leaders in the field. These include Tesla Optimus, Figure AI, and Boston Dynamics' electric Atlas. These machines are designed to mimic human morphology to operate in human-centric environments, such as factories and homes.

2025 تک، کئی انسان نما (humanoid) پلیٹ فارمز اس شعبے میں لیڈر کے طور پر ابھرے ہیں۔ ان میں Tesla Optimus، Figure AI، اور Boston Dynamics کا الیکٹرک Atlas شامل ہیں۔ یہ مشینیں انسانی ساخت کی نقل کرنے کے لیے بنائی گئی ہیں تاکہ وہ انسانوں کے بنائے ہوئے ماحول، جیسے فیکٹریوں اور گھروں میں کام کر سکیں۔

## 1.4 ROS 2 Fundamentals
### (1.4 کے بنیادی اصول ROS 2)

The Robot Operating System 2 (ROS 2) is the industry-standard middleware. It organizes the robot's software into **Nodes** [نوڈ]. These nodes communicate via **Topics** [موضوع] using a **Publisher/Subscriber** [شاہکار/صارف] model.

روبوٹ آپریٹنگ سسٹم 2 (ROS 2) صنعت کا معیاری مڈل وئیر ہے۔ یہ روبوٹ کے سافٹ ویئر کو **نوڈز** (Nodes) میں ترتیب دیتا ہے۔ یہ نوڈس ایک **شاہکار/صارف** (Publisher/Subscriber) ماڈل کا استعمال کرتے ہوئے **موضوعات** (Topics) کے ذریعے ایک دوسرے سے رابطہ کرتے ہیں۔

```bash
ros2 topic list
```

## 1.5 Why Hands-On Labs Matter
### (1.5 عملی تجربہ گاہوں کی اہمیت)

Theory alone cannot build a robot. You must compile code, debug hardware, and see your algorithms fail in simulation before they succeed in reality. This book focuses on hands-on labs using Gazebo and ROS 2 Humble to ensure you gain practical experience.

صرف نظریہ (theory) ایک روبوٹ نہیں بنا سکتا۔ آپ کو کوڈ کمپائل کرنا ہوگا، ہارڈ ویئر کو ڈیبگ کرنا ہوگا، اور حقیقت میں کامیاب ہونے سے پہلے سیمولیشن میں اپنے الگورتھمز کو ناکام ہوتے دیکھنا ہوگا۔ یہ کتاب Gazebo اور ROS 2 Humble کا استعمال کرتے ہوئے عملی تجربہ گاہوں پر توجہ مرکوز کرتی ہے تاکہ آپ عملی تجربہ حاصل کر سکیں۔
"""

# Manual splitting for high semantic quality
sections = {
    "intro": "The 'Glass Wall' of AI and Disembodied AI concept.",
    "sec1_1": "1.1 What is Embodied Intelligence? Definition and Sensorimotor Loop.",
    "sec1_2": "1.2 The Simulation-to-Real Gap and simulation training challenges.",
    "sec1_3": "1.3 Current Humanoid Platforms (2025): Optimus, Figure, Atlas.",
    "sec1_4": "1.4 ROS 2 Fundamentals: Nodes, Topics, Pub/Sub.",
    "sec1_5": "1.5 Why Hands-On Labs Matter: Practical experience with Gazebo."
}

# --- Add Chapter Chunks ---
draft_segments = chapter_content.split('---') # Though I'll do it manually for precision
# I'll just use the content directly from my parsed logic.

# (Drafting chunk metadata logic)
def add_chunk(content, section, content_type="theory", difficulty="beginner", keywords=[], source="chapter-1-urdu.md", position=0):
    chunk = {
        "chunk_id": generate_chunk_id(section, len(chunks)),
        "chapter": CHAPTER_NUMBER,
        "chapter_title": CHAPTER_TITLE,
        "section": section,
        "section_number": position,
        "content": content.strip(),
        "content_type": content_type,
        "difficulty": difficulty,
        "keywords": keywords,
        "timestamp": "2025-12-31T00:00:00Z",
        "source": source,
        "position": position,
        "estimated_read_time_seconds": (len(content) // 10) # rough estimate
    }
    chunks.append(chunk)

# Chunks from Chapter Draft
add_chunk(
    "Introduction: The 'Glass Wall' of AI. For the last decade, AI development has largely happened behind a 'glass wall.' Disembodied AI is brilliant logic trapped in a box without hands or feet. (Urdu: تعارف: AI کی 'شفاف دیوار')",
    "Introduction", "theory", "beginner", ["disembodied AI", "glass wall"], "chapter-1-urdu.md", 0
)
add_chunk(
    "1.1 What is Embodied Intelligence? Embodied Intelligence refers to AI that is grounded in a physical form. It requires a Sensorimotor Loop: sensors (cameras, LiDAR) perceive, AI decides, actuators execute. (Urdu: جسمانی ذہانت کیا ہے؟)",
    "What is Embodied Intelligence?", "theory", "beginner", ["embodied intelligence", "sensorimotor loop", "actuators"], "chapter-1-urdu.md", 1
)
add_chunk(
    "1.2 The Simulation-to-Real Gap. Friction, lighting, and gravity in the real world are never exactly as they appear in simulator code like Gazebo. (Urdu: نقالی سے حقیقی فاصلہ)",
    "The Simulation-to-Real Gap", "theory", "beginner", ["sim-to-real", "Gazebo", "simulation"], "chapter-1-urdu.md", 2
)
add_chunk(
    "1.3 Current Humanoid Platforms (2025). Leaders include Tesla Optimus, Figure AI, and Boston Dynamics' electric Atlas. (Urdu: موجودہ انسان نما پلیٹ فارمز)",
    "Current Humanoid Platforms", "theory", "beginner", ["humanoid", "Tesla Optimus", "Figure AI", "Atlas"], "chapter-1-urdu.md", 3
)
add_chunk(
    "1.4 ROS 2 Fundamentals. ROS 2 middleware organizes software into Nodes communicating via Topics using a Publisher/Subscriber model. (Urdu: ROS 2 کے بنیادی اصول)",
    "ROS 2 Fundamentals", "theory", "intermediate", ["ROS 2", "Nodes", "Topics", "Pub/Sub"], "chapter-1-urdu.md", 4
)

# --- Add Lab Chunks ---
lab_content = """# Lab 1: Hello Physical AI: Your First Sensorimotor Loop
In this lab, you create a control system that reads position data (Sense), processes it (Think), and sends commands (Act).
Setup: run 'make up' to start Gazebo.
Implementation: Create a node in 'src/hello_physical_ai_node.py' subscribing to '/turtle1/pose' and publishing to '/turtle1/cmd_vel'.
Verification: Robot crosses X=2.0 and reverses.
"""
add_chunk(lab_content, "Lab 1: Hello Physical AI", "lab", "beginner", ["lab", "sensorimotor loop", "hands-on"], "lab/README.md", 5)

# --- Add Code Chunk ---
code_snippet = """
class SensorimotorLoopNode(Node):
    def __init__(self):
        self.cmd_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
    def pose_callback(self, msg):
        if msg.x > threshold:
            cmd.linear.x = -1.0
        else:
            cmd.linear.x = 1.0
        self.cmd_publisher.publish(cmd)
"""
add_chunk(code_snippet, "Implementation: Node Logic", "code", "intermediate", ["python", "ROS 2", "code"], "lab/src/hello_physical_ai_node.py", 6)

# --- Add Assessment Chunks ---
add_chunk(
    "Q: What is the primary difference between an LLM and a VLA model? A: VLA (Vision-Language-Action) models include physical action outputs for embodied systems.",
    "Assessment: Conceptual", "qa", "beginner", ["assessment", "VLA", "LLM"], "assessments.md", 7
)
add_chunk(
    "Q: In ROS 2, what is a 'Topic'? A: An information channel where nodes broadcast and consume data asynchronously.",
    "Assessment: ROS 2", "qa", "beginner", ["assessment", "ROS 2", "topic"], "assessments.md", 8
)

# --- Add Diagram Chunks ---
diagram_1 = "Figure 1.1: Sensorimotor Loop. Circular relationship between World -> Sensors -> Brain -> Actuators -> World."
add_chunk(diagram_1, "Visuals: Sensorimotor Loop", "diagram", "beginner", ["diagram", "feedback loop"], "diagrams.md", 9)

# --- Add Personalization Variants ---
add_chunk(
    "Beginner Concept: Think of embodied intelligence like a thermostat: senses temp, triggers furnace, feels warmth. An egg analogy: you feel the weight and adjust grip instinctively.",
    "Personalization: Beginner", "theory", "beginner", ["analogy", "thermostat", "egg analogy"], "personalization.md", 10
)
add_chunk(
    "Intermediate Concept: Formalized loop Perception(t) -> Action(t) -> World Response(t+dt). Domain Randomization: sampling friction from a distribution to ensure robustness.",
    "Personalization: Intermediate", "theory", "intermediate", ["math", "feedback loop", "robustness"], "personalization.md", 11
)

# --- Export JSONL ---
with open(os.path.join(OUTPUT_DIR, "chunks.jsonl"), "w", encoding="utf-8") as f:
    for chunk in chunks:
        f.write(json.dumps(chunk, ensure_ascii=False) + "\n")

# --- Export Manifest ---
manifest = {
    "chapter": CHAPTER_NUMBER,
    "chapter_title": CHAPTER_TITLE,
    "total_chunks": len(chunks),
    "total_tokens": sum(len(c["content"]) // 4 for c in chunks),
    "embedding_model": "text-embedding-3-small",
    "embedding_dimension": 1536,
    "created_at": datetime.utcnow().isoformat() + "Z",
    "chunks": [
        {
            "chunk_id": c["chunk_id"],
            "section": c["section"],
            "content_preview": c["content"][:100] + "...",
            "tokens": len(c["content"]) // 4,
            "difficulty": c["difficulty"],
            "keywords": c["keywords"]
        } for c in chunks
    ]
}

with open(os.path.join(OUTPUT_DIR, "rag-manifest.json"), "w", encoding="utf-8") as f:
    json.dump(manifest, f, indent=2, ensure_ascii=False)

print(f"Index complete. {len(chunks)} chunks generated.")
