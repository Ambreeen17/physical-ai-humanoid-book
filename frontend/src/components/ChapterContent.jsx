import React, { useState, useEffect } from 'react';
import { useTranslation } from './TranslationProvider';

// Chapter translations - Urdu versions
const CHAPTER_TRANSLATIONS = {
  ur: {
    1: `
# باب 1: فزیکل AI کا تعارف

## سیکھنے کے مقاصد
اس باب کے اختتام پر، آپ قابل ہوں گے:
1. **مجسم ذہانت کی تعریف** کریں اور بتائیں کہ یہ صرف ڈیجیٹل AI سسٹمز سے کیسے مختلف ہے
2. **سینسریموٹر لوپ کو سمجھیں** اور یہ کیوں فزیکل انٹیلیجنس کی بنیاد ہے
3. **سمولیشن سے حقیقت کے فرق کی شناخت** کریں اور اسے پر کرنے کی تین حکمت عملیاں بیان کریں
4. **بڑے ہیومنائیڈ پلیٹ فارمز کو پہچانیں** اور 2025 میں ان کی ایپلیکیشنز
5. **ROS 2 pub/sub کمیونیکیشن استعمال کریں** سمولیٹڈ روبوٹ کو کنٹرول کرنے کے لیے

---

## تعارف

پچھلی دہائی میں، AI کی ترقی زیادہ تر سکرینوں کے پیچھے ہوئی ہے۔ ChatGPT جیسے بڑے لینگویج ماڈلز مضامین اور کوڈ لکھ سکتے ہیں، لیکن وہ کافی کا کپ نہیں اٹھا سکتے یا سیڑھیوں پر نہیں چل سکتے۔ ایک پوشیدہ دیوار ہے—اسے **شیشے کی دیوار** کہیں—جو ڈیجیٹل ذہانت کو فزیکل ذہانت سے الگ کرتی ہے۔

یہ باب اس دیوار کو ہٹاتا ہے۔

فزیکل AI وہ ذہانت ہے جو فزیکل سسٹم میں مجسم ہوتی ہے جو براہ راست حقیقی دنیا سے تعامل کرتی ہے۔

---

## سیکشن 1.1: مجسم ذہانت

### مجسم ذہانت کیا ہے؟

**مجسم ذہانت** وہ اصول ہے کہ ذہانت تین عناصر کے درمیان متحرک تعامل سے ابھرتی ہے:
- **دماغ**: الگورتھمز، نیورل نیٹ ورکس، فیصلہ سازی کی منطق
- **جسم**: ساخت، جوڑوں کی ساخت، حرکت کی صلاحیتیں
- **ماحول**: فزکس، کشش ثقل، رگڑ، روشنی، رکاوٹیں

### سینسریموٹر لوپ

مجسم ذہانت کا دل **سینسریموٹر لوپ** ہے:

\`\`\`
ادراک → فیصلہ → عمل → نیا ادراک → ...
\`\`\`

یہ سائیکل ہر سیکنڈ سینکڑوں بار چلتا ہے۔
`,
    2: `
# باب 2: کائنیمیٹکس اور موشن

## سیکھنے کے مقاصد
1. فارورڈ کائنیمیٹکس کو سمجھیں
2. انورس کائنیمیٹکس کا اطلاق کریں
3. DH پیرامیٹرز استعمال کریں

## تعارف
کائنیمیٹکس روبوٹ کی حرکت کا مطالعہ ہے۔
`,
  },
  zh: {
    1: `
# 第1章：物理AI简介

## 学习目标
完成本章后，您将能够：
1. **定义具身智能**并解释它与纯数字AI系统的区别
2. **理解感知-运动循环**及其对物理智能的重要性
3. **识别仿真到现实的差距**并描述三种弥合策略
4. **认识主要人形机器人平台**及其2025年的应用
5. **使用ROS 2发布/订阅通信**控制模拟机器人

---

## 简介

在过去十年中，AI开发主要发生在屏幕后面。像ChatGPT这样的大型语言模型可以撰写文章和代码，但它们无法拿起一杯咖啡或在楼梯上行走。

物理AI是嵌入物理系统中的智能，直接与现实世界交互。

## 第1.1节：具身智能

### 什么是具身智能？

**具身智能**是智能从三个要素的动态交互中涌现的原理：
- **大脑**：算法、神经网络、决策逻辑
- **身体**：形态、关节结构、执行能力
- **环境**：物理、重力、摩擦、光线、障碍物
`,
  },
  es: {
    1: `
# Capítulo 1: Introducción a la IA Física

## Objetivos de Aprendizaje
Al final de este capítulo, podrás:
1. **Definir la inteligencia encarnada** y explicar cómo difiere de los sistemas de IA solo digitales
2. **Comprender el bucle sensoriomotor** y por qué es fundamental para la inteligencia física
3. **Identificar la brecha simulación-realidad** y describir tres estrategias para superarla
4. **Reconocer las principales plataformas humanoides** y sus aplicaciones en 2025
5. **Usar la comunicación pub/sub de ROS 2** para controlar un robot simulado

---

## Introducción

Durante la última década, el desarrollo de IA ha ocurrido principalmente detrás de pantallas. Los modelos de lenguaje grandes como ChatGPT pueden escribir ensayos y código, pero no pueden recoger una taza de café o navegar por una escalera.

La IA Física es inteligencia incorporada en un sistema físico que interactúa directamente con el mundo real.

## Sección 1.1: Inteligencia Encarnada

### ¿Qué es la Inteligencia Encarnada?

**La inteligencia encarnada** es el principio de que la inteligencia emerge de la interacción dinámica entre tres elementos:
- **El Cerebro**: Algoritmos, redes neuronales, lógica de decisión
- **El Cuerpo**: Morfología, estructura articular, capacidades de actuación
- **El Entorno**: Física, gravedad, fricción, luz, obstáculos
`,
  },
  ar: {
    1: `
# الفصل 1: مقدمة في الذكاء الاصطناعي الفيزيائي

## أهداف التعلم
بنهاية هذا الفصل، ستتمكن من:
1. **تعريف الذكاء المجسد** وشرح كيف يختلف عن أنظمة الذكاء الاصطناعي الرقمية فقط
2. **فهم الحلقة الحسية الحركية** ولماذا هي أساسية للذكاء الفيزيائي
3. **تحديد فجوة المحاكاة إلى الواقع** ووصف ثلاث استراتيجيات لسدها

## مقدمة

خلال العقد الماضي، حدث تطوير الذكاء الاصطناعي إلى حد كبير خلف الشاشات.

الذكاء الاصطناعي الفيزيائي هو ذكاء متجسد في نظام فيزيائي يتفاعل مباشرة مع العالم الحقيقي.
`,
  },
};

/**
 * ChapterContent - Displays chapter content in selected language
 */
const ChapterContent = ({ chapterId, children }) => {
  const { language } = useTranslation();
  const [translatedContent, setTranslatedContent] = useState(null);

  useEffect(() => {
    if (language !== 'en' && CHAPTER_TRANSLATIONS[language]?.[chapterId]) {
      setTranslatedContent(CHAPTER_TRANSLATIONS[language][chapterId]);
    } else {
      setTranslatedContent(null);
    }
  }, [language, chapterId]);

  // If we have translated content, show it
  if (translatedContent) {
    return (
      <div className={language === 'ur' || language === 'ar' ? 'rtl-content' : ''}>
        <div
          className="translated-chapter"
          style={{
            direction: language === 'ur' || language === 'ar' ? 'rtl' : 'ltr',
            textAlign: language === 'ur' || language === 'ar' ? 'right' : 'left',
          }}
        >
          <div className="translation-notice" style={{
            background: '#e3f2fd',
            padding: '10px 15px',
            borderRadius: '5px',
            marginBottom: '20px',
            fontSize: '14px'
          }}>
            {language === 'ur' ? '🌐 یہ مواد اردو میں دکھایا جا رہا ہے' :
             language === 'zh' ? '🌐 此内容以中文显示' :
             language === 'es' ? '🌐 Este contenido se muestra en español' :
             language === 'ar' ? '🌐 يتم عرض هذا المحتوى باللغة العربية' :
             '🌐 Content displayed in selected language'}
          </div>
          <div dangerouslySetInnerHTML={{ __html: markdownToHtml(translatedContent) }} />
        </div>
      </div>
    );
  }

  // Default: show original English content
  return children;
};

// Simple markdown to HTML converter
function markdownToHtml(md) {
  return md
    .replace(/^### (.*$)/gim, '<h3>$1</h3>')
    .replace(/^## (.*$)/gim, '<h2>$1</h2>')
    .replace(/^# (.*$)/gim, '<h1>$1</h1>')
    .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
    .replace(/\*(.*?)\*/g, '<em>$1</em>')
    .replace(/```([\s\S]*?)```/g, '<pre><code>$1</code></pre>')
    .replace(/`(.*?)`/g, '<code>$1</code>')
    .replace(/^- (.*$)/gim, '<li>$1</li>')
    .replace(/^\d+\. (.*$)/gim, '<li>$1</li>')
    .replace(/\n\n/g, '</p><p>')
    .replace(/^---$/gm, '<hr/>')
    .replace(/\n/g, '<br/>');
}

export default ChapterContent;
