"""Localization Agent - Creates localized content for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class LocalizationAgent(AgentInterface):
    """Localization agent that creates localized content for the chapter (Urdu)."""

    def __init__(self):
        super().__init__("Localization")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute localization for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with localization artifacts
        """
        self.logger.info(f"Starting localization for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Get chapter content if available from previous artifacts
            chapter_content = ""
            if agent_input.previous_artifacts and "chapter-draft.md" in agent_input.previous_artifacts:
                chapter_path = agent_input.previous_artifacts["chapter-draft.md"]
                try:
                    with open(chapter_path, 'r', encoding='utf-8') as f:
                        chapter_content = f.read()
                except FileNotFoundError:
                    self.logger.warning(f"Chapter file not found: {chapter_path}")

            # Generate localization content
            localization_content = self._generate_localization_content(agent_input, chapter_content)
            
            # Write localization file
            localization_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/chapter-urdu.md"
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(localization_file_path), exist_ok=True)
            
            with open(localization_file_path, 'w', encoding='utf-8') as f:
                f.write(localization_content)
            
            # Generate glossary
            glossary_content = self._generate_glossary(agent_input)
            glossary_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/glossary.json"
            
            with open(glossary_file_path, 'w', encoding='utf-8') as f:
                f.write(glossary_content)
            
            # Generate style guide
            style_guide_content = self._generate_style_guide(agent_input)
            style_guide_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/style_guide.md"
            
            with open(style_guide_file_path, 'w', encoding='utf-8') as f:
                f.write(style_guide_content)
            
            artifacts = {
                "chapter-urdu.md": localization_file_path,
                "glossary.json": glossary_file_path,
                "style_guide.md": style_guide_file_path
            }
            
            self.logger.info(f"Localization completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=25  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"Localization agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _generate_localization_content(self, agent_input: AgentInput, chapter_content: str) -> str:
        """Generate localization content based on chapter metadata and content."""
        content = f"""# فصل {agent_input.chapter_number}: {agent_input.chapter_topic} - {agent_input.chapter_topic}

## سیکھنے کے اہداف

یہ فصل مکمل کرنے کے بعد، آپ کو اس قابل ہو گا:

1. {agent_input.chapter_topic} سے متعلق کلیدی تصورات کی وضاحت کرنا
2. روبوٹکس کے مسائل پر {agent_input.chapter_topic} کے اصول لاگو کرنا
3. ROS 2 میں {agent_input.chapter_topic} کے حلات کا نفاذ کرنا
4. {agent_input.chapter_topic} کی کارکردگی کا جائزہ لینا
5. دیگر روبوٹکس سسٹمز کے ساتھ {agent_input.chapter_topic} کا اTEGRیشن کرنا

## تعارف

{agent_input.chapter_topic} روبوٹکس کا ایک بنیادی تصور ہے جو روبوٹس کو اپنے ماحول کے ساتھ مؤثر طریقے سے بات چیت کرنے کے قابل بناتا ہے۔ یہ فصل {agent_input.chapter_topic} کے نظریاتی بنیادوں، عملی اطلاقوں، اور حقیقی دنیا کے اطلاقوں کا جائزہ لیتی ہے۔

## 1.1 {agent_input.chapter_topic} کا جائزہ

{agent_input.chapter_topic} کے میدان میں گزشتہ دہائی میں کافی ترقی ہوئی ہے۔ جدید نقطہ نظر کمپیوٹیشنل طاقت، سینسر ٹیکنالوجی، اور الگورتھم کی کارکردگی کے فوائد کو استعمال کرتے ہیں تاکہ حقیقی دنیا کے روبوٹکس اطلاقوں میں بے مثال کارکردگی حاصل کی جا سکے۔

### تاریخی پس منظر

{agent_input.chapter_topic} کے ابتدائی نقطہ نظر کمپیوٹیشنل رکاوٹوں اور سینسر کی درستگی کی وجہ سے محدود تھے۔ تاہم، حالیہ ترقیات نے متحرک، غیر منظم ماحول میں کام کرنے کے قابل زیادہ جامع اطلاقوں کو فروغ دیا ہے۔

### موجودہ حالت-کے-فن

2025 کے مطابق، {agent_input.chapter_topic} میں موجودہ حالت-کے-فن میں شامل ہیں:

- اعلی کمپیوٹیشنل طریقے
- حقیقی وقت کی پروسیسنگ کی صلاحیتیں
- ماحولیاتی متغیرات کے لیے مضبوطی
- دیگر روبوٹکس ذیلی نظام کے ساتھ اTEGRیشن

## 1.2 نظریاتی بنیادیں

{agent_input.chapter_topic} کی نظریاتی بنیاد کئی کلیدی اصولوں پر مبنی ہے:

### ریاضی کا ڈھانچہ

{agent_input.chapter_topic} کے لیے ریاضی کا ڈھانچہ عام طور پر ان میں شامل ہے:

- جگہی تبدیلیوں کے لیے لکیری الجبرا
- حرکت کے تجزیے کے لیے کیلکولس
- عدم یقینی کو سنبھالنے کے لیے احتمال کا نظریہ
- کارکردگی کو زیادہ سے زیادہ کرنے کے لیے اصلاح کے طریقے

### کلیدی مساواتیں

{agent_input.chapter_topic} کو متعین کرنے والی بنیادی مساواتیں شامل ہیں:

```math
% کلیدی مساواتوں کے لیے جگہ
% یہ فصل کے موضوع کے مطابق ہوں گے
```

## 1.3 ہارڈ ویئر کے اہتمام

حقیقی ہارڈ ویئر پر {agent_input.chapter_topic} کا نفاذ کرتے وقت کئی عوامل کو مدنظر رکھا جانا چاہیے:

### یونٹری جی1 کی خصوصیات

یونٹری جی1 روبوٹ {agent_input.chapter_topic} سے متعلق مخصوص صلاحیتیں فراہم کرتا ہے:

- جوڑ کی ترتیب اور حرکت کی حد
- سینسر کا اTEGRیشن اور ڈیٹا کی شرح
- کمپیوٹیشنل وسائل اور حقیقی وقت کی رکاوٹیں
- بجلی کی کھپت اور حرارت کا انتظام

### سینسر کا اTEGRیشن

{agent_input.chapter_topic} کے موثر اTEGRیشن کے لیے مختلف سینسرز کے ساتھ دیکھ بھال کی جانی چاہیے:

- پوزیشن اور جہت کے سینسرز
- قوت اور ٹورک سینسرز
- ماحولیاتی ادراک کے سینسرز
- رابطے کے واسطے

## 1.4 سافٹ ویئر کا نفاذ

ROS 2 میں {agent_input.chapter_topic} کا سافٹ ویئر نفاذ کئی کلیدی اجزاء پر مشتمل ہے:

### نوڈ آرکیٹیکچر

تجویز کردہ نوڈ آرکیٹیکچر میں شامل ہے:

- ڈیٹا کے بہاؤ کے لیے شائع کنندہ/Subscriber پیٹرنز
- ہم وقتی کاموں کے لیے سروس کالز
- طویل وقت تک چلنے والے کاموں کے لیے ایکشن سرورز
- کنفیگریشن کے لیے پیرامیٹر کا انتظام

### کوڈ کی مثال

```python
# ROS 2 میں {agent_input.chapter_topic} کے نفاذ کی مثال
import rclpy
from rclpy.node import Node

class {agent_input.chapter_topic.replace(' ', '')}Node(Node):
    def __init__(self):
        super().__init__('{agent_input.chapter_topic.replace(' ', '_').lower()}_node')
        # {agent_input.chapter_topic} کے اجزاء کو شروع کریں
        self.get_logger().info('{agent_input.chapter_topic} نوڈ شروع کیا گیا')
    
    def process_{agent_input.chapter_topic.replace(' ', '_').lower()}(self):
        # {agent_input.chapter_topic} کے منطق کا نفاذ
        pass

def main(args=None):
    rclpy.init(args=args)
    node = {agent_input.chapter_topic.replace(' ', '')}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 1.5 شبیہ سازی اور ٹیسٹنگ

{agent_input.chapter_topic} کے اطلاقوں کو تیار کرنے اور جانچنے میں شبیہ سازی ایک اہم کردار ادا کرتی ہے:

### شبیہ سازی کے ماحول

{agent_input.chapter_topic} کے لیے مندرجہ ذیل شبیہ سازی کے ماحول تجویز کیے گئے ہیں:

- زیادہ معیاری فزکس کی شبیہ سازی کے لیے MuJoCo
- حقیقی سینسر کی شبیہ سازی کے لیے Gazebo
- تیز ترین پروٹو ٹائپنگ کے لیے PyBullet

### ٹیسٹنگ کی حکمت عملیاں

{agent_input.chapter_topic} کے اطلاقوں کی مؤثر ٹیسٹنگ میں شامل ہونا چاہیے:

- انفرادی اجزاء کے لیے یونٹ ٹیسٹس
- مکمل سسٹمز کے لیے اTEGRیشن ٹیسٹس
- کارکردگی کے بینچ مارکس
- مضبوطی کی توثیق

## 1.6 عملی اطلاق

{agent_input.chapter_topic} کئی روبوٹکس کے شعبوں میں اطلاق پاتا ہے:

- مینیپولیشن اور گریسنگ
- نیویگیشن اور راستہ کی منصوبہ بندی
- انسان-روبوٹ بات چیت
- خودکار سسٹمز

## 1.7 چیلنجز اور حدود

نمایاں ترقی کے باوجود، {agent_input.chapter_topic} کو کئی چیلنجز کا سامنا ہے:

- کمپیوٹیشنل پیچیدگی
- حقیقی وقت کی کارکردگی کی ضروریات
- ماحولیاتی عدم یقینی
- ہارڈ ویئر کی حدود

## خلاصہ

اس فصل میں {agent_input.chapter_topic} کے بنیادی تصورات کو پیش کیا گیا ہے، بشمول نظریاتی بنیادیں، عملی اطلاق، اور حقیقی دنیا کے اطلاق۔ اگلی فصل اس کے تصورات کی بنیاد پر مزید اعلی موضوعات کا جائزہ لے گی۔

## مشقیں

1. ROS 2 میں {agent_input.chapter_topic} کا بنیادی الگورتھم نافذ کریں
2. مختلف حالات کے تحت اپنے نفاذ کی کارکردگی کا تجزیہ کریں
3. متبادل طریقوں کے ساتھ اپنے نقطہ نظر کا موازنہ کریں
4. اپنے جائزے دستاویز کریں اور بہتریوں کی تجویز کریں

---

**تاریخ تیار کردہ**: {datetime.now().isoformat()}
**فصل**: {agent_input.chapter_number} - {agent_input.chapter_topic}
"""
        return content

    def _generate_glossary(self, agent_input: AgentInput) -> str:
        """Generate glossary for the chapter."""
        glossary = {
            "chapter": agent_input.chapter_number,
            "topic": agent_input.chapter_topic,
            "glossary": {
                "robotics": {
                    "english": "Robotics",
                    "urdu": "روبوٹکس",
                    "definition": "The interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others."
                },
                "ros": {
                    "english": "ROS",
                    "urdu": "ROS",
                    "definition": "Robot Operating System - a flexible framework for writing robot software."
                },
                f"{agent_input.chapter_topic.lower().replace(' ', '_')}": {
                    "english": agent_input.chapter_topic,
                    "urdu": agent_input.chapter_topic,
                    "definition": f"The main topic of Chapter {agent_input.chapter_number}, fundamental to robotics applications."
                },
                "sensor": {
                    "english": "Sensor",
                    "urdu": "سینسر",
                    "definition": "A device that detects and responds to some type of input from the physical environment."
                },
                "actuator": {
                    "english": "Actuator",
                    "urdu": "اکچویٹر",
                    "definition": "A component of a machine that is responsible for moving and controlling a mechanism."
                },
                "kinematics": {
                    "english": "Kinematics",
                    "urdu": "کنیمیٹکس",
                    "definition": "The branch of mechanics that deals with pure motion, without reference to force or mass."
                },
                "dynamics": {
                    "english": "Dynamics",
                    "urdu": "ڈائنامکس",
                    "definition": "The branch of mechanics concerned with the motion of bodies under the action of forces."
                },
                "unitree_g1": {
                    "english": "Unitree G1",
                    "urdu": "یونٹری جی1",
                    "definition": "An advanced humanoid robot platform used for research and development in robotics."
                },
                "mujoco": {
                    "english": "MuJoCo",
                    "urdu": "MuJoCo",
                    "definition": "Multi-Joint dynamics with Contact - a physics engine for detailed, accurate simulations."
                },
                "gazebo": {
                    "english": "Gazebo",
                    "urdu": "گزیبو",
                    "definition": "A 3D simulation environment for robotics applications."
                }
            },
            "generated_at": datetime.now().isoformat()
        }
        import json
        return json.dumps(glossary, ensure_ascii=False, indent=2)

    def _generate_style_guide(self, agent_input: AgentInput) -> str:
        """Generate style guide for localization."""
        return f"""# Urdu Localization Style Guide for Chapter {agent_input.chapter_number}

## General Guidelines

1. **Maintain Technical Accuracy**: Preserve technical terminology while making it accessible in Urdu
2. **Code and Equations**: Keep code snippets, mathematical equations, and tool names in English
3. **Transliteration**: Use consistent transliteration for technical terms not commonly used in Urdu
4. **Cultural Sensitivity**: Ensure examples and analogies are culturally appropriate

## Translation Principles

### Technical Terms
- Keep ROS, Python, C++, and other programming language keywords in English
- Preserve mathematical notation and equations
- Maintain consistency in technical terminology throughout the chapter

### Code Examples
- Keep all code examples in English
- Add Urdu comments where appropriate to explain concepts
- Maintain original formatting and structure

### Examples and Analogies
- Adapt examples to be culturally relevant
- Use analogies that resonate with Urdu-speaking audience
- Maintain educational value while improving accessibility

## Specific Terms for Chapter {agent_input.chapter_number}

### {agent_input.chapter_topic} Related Terms
- {agent_input.chapter_topic} - {agent_input.chapter_topic}
- Algorithm - الگورتھم
- Implementation - نفاذ
- Simulation - شبیہ سازی
- Hardware - ہارڈ ویئر
- Software - سافٹ ویئر
- Node - نوڈ
- Topic - ٹوپک
- Publisher - شائع کنندہ
- Subscriber - سبسکرائب کنندہ

## Formatting Guidelines

### Headers
- Maintain original header structure
- Translate header text appropriately
- Preserve numbering and hierarchy

### Lists and Tables
- Keep original structure
- Translate content while preserving technical accuracy
- Maintain readability in Urdu

### Code Blocks
- Do not translate code
- Add Urdu explanations as needed
- Preserve original formatting

## Quality Assurance

### Review Process
1. Technical accuracy review
2. Linguistic accuracy review
3. Cultural appropriateness review
4. Consistency check

### Validation Criteria
- Technical concepts accurately conveyed
- Language appropriate for target audience
- Cultural sensitivity maintained
- Consistency with previous chapters

---

**Style guide generated**: {datetime.now().isoformat()}
**Chapter**: {agent_input.chapter_number} - {agent_input.chapter_topic}
"""