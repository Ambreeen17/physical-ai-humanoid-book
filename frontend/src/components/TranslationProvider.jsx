import React, { createContext, useContext, useState, useCallback } from 'react';

// Translation context
const TranslationContext = createContext();

// Pre-built translations for common UI elements (instant, no API needed)
const UI_TRANSLATIONS = {
  en: {
    dashboard: 'Learner Dashboard',
    profile: 'Your Profile',
    difficulty: 'Difficulty Level',
    progress: 'Progress Overview',
    assessments: 'Recent Assessment Results',
    score: 'Score',
    passed: 'PASSED',
    failed: 'FAILED',
    feedback: 'Feedback',
    continue: 'Continue Learning',
    editProfile: 'Edit Profile',
    totalAssessments: 'Total Assessments',
    averageScore: 'Average Score',
    experienceScores: 'Experience Scores',
    recommendedChapters: 'Recommended Chapters',
    python: 'Python',
    ml: 'ML/AI',
    robotics: 'Robotics',
    ros: 'ROS 2',
    beginner: 'Beginner',
    intermediate: 'Intermediate',
    advanced: 'Advanced',
    demoMode: 'Demo Mode - Backend not connected. Showing sample data.',
    noResults: 'No assessment results yet. Complete assessments to see them here.',
    chatPlaceholder: 'Ask about ROS 2, embodied AI, sim-to-real gap, labs...',
    chatWelcome: "Hello! I'm your Physical AI learning assistant. Ask me anything about embodied intelligence, ROS 2, humanoid robots, or the course content.",
    thinking: 'Thinking...',
    online: 'Online',
    clearChat: 'Clear conversation',
    send: 'Send',
    selectLanguage: 'Select Language',
    chapters: 'Chapters',
    aiAssistant: 'AI Assistant',
  },
  ur: {
    dashboard: 'سیکھنے والے کا ڈیش بورڈ',
    profile: 'آپ کی پروفائل',
    difficulty: 'مشکل کی سطح',
    progress: 'پیش رفت کا جائزہ',
    assessments: 'حالیہ تشخیص کے نتائج',
    score: 'اسکور',
    passed: 'کامیاب',
    failed: 'ناکام',
    feedback: 'رائے',
    continue: 'سیکھنا جاری رکھیں',
    editProfile: 'پروفائل میں ترمیم کریں',
    totalAssessments: 'کل تشخیصات',
    averageScore: 'اوسط اسکور',
    experienceScores: 'تجربے کے اسکور',
    recommendedChapters: 'تجویز کردہ ابواب',
    python: 'پائتھون',
    ml: 'مشین لرننگ',
    robotics: 'روبوٹکس',
    ros: 'آر او ایس 2',
    beginner: 'ابتدائی',
    intermediate: 'درمیانی',
    advanced: 'ایڈوانسڈ',
    demoMode: 'ڈیمو موڈ - بیک اینڈ منسلک نہیں۔ نمونہ ڈیٹا دکھا رہا ہے۔',
    noResults: 'ابھی تک کوئی تشخیص کے نتائج نہیں۔',
    chatPlaceholder: 'ROS 2، روبوٹکس، یا کورس کے بارے میں پوچھیں...',
    chatWelcome: 'سلام! میں آپ کا فزیکل AI سیکھنے کا معاون ہوں۔',
    thinking: 'سوچ رہا ہوں...',
    online: 'آن لائن',
    clearChat: 'بات چیت صاف کریں',
    send: 'بھیجیں',
    selectLanguage: 'زبان منتخب کریں',
    chapters: 'ابواب',
    aiAssistant: 'AI معاون',
  },
  zh: {
    dashboard: '学习者仪表板',
    profile: '您的个人资料',
    difficulty: '难度级别',
    progress: '进度概览',
    assessments: '最近的评估结果',
    score: '分数',
    passed: '通过',
    failed: '未通过',
    feedback: '反馈',
    continue: '继续学习',
    editProfile: '编辑个人资料',
    totalAssessments: '总评估数',
    averageScore: '平均分',
    experienceScores: '经验分数',
    recommendedChapters: '推荐章节',
    python: 'Python',
    ml: '机器学习/AI',
    robotics: '机器人学',
    ros: 'ROS 2',
    beginner: '初级',
    intermediate: '中级',
    advanced: '高级',
    demoMode: '演示模式 - 后端未连接。显示示例数据。',
    noResults: '暂无评估结果。完成评估后将在此显示。',
    chatPlaceholder: '询问ROS 2、机器人、课程内容...',
    chatWelcome: '你好！我是您的物理AI学习助手。',
    thinking: '思考中...',
    online: '在线',
    clearChat: '清除对话',
    send: '发送',
    selectLanguage: '选择语言',
    chapters: '章节',
    aiAssistant: 'AI助手',
  },
  es: {
    dashboard: 'Panel del Estudiante',
    profile: 'Tu Perfil',
    difficulty: 'Nivel de Dificultad',
    progress: 'Resumen de Progreso',
    assessments: 'Resultados Recientes',
    score: 'Puntuación',
    passed: 'APROBADO',
    failed: 'REPROBADO',
    feedback: 'Comentarios',
    continue: 'Continuar Aprendiendo',
    editProfile: 'Editar Perfil',
    totalAssessments: 'Total de Evaluaciones',
    averageScore: 'Puntuación Promedio',
    experienceScores: 'Puntuaciones de Experiencia',
    recommendedChapters: 'Capítulos Recomendados',
    python: 'Python',
    ml: 'ML/IA',
    robotics: 'Robótica',
    ros: 'ROS 2',
    beginner: 'Principiante',
    intermediate: 'Intermedio',
    advanced: 'Avanzado',
    demoMode: 'Modo Demo - Backend no conectado. Mostrando datos de ejemplo.',
    noResults: 'Sin resultados de evaluación. Completa evaluaciones para verlas aquí.',
    chatPlaceholder: 'Pregunta sobre ROS 2, robótica, cursos...',
    chatWelcome: '¡Hola! Soy tu asistente de aprendizaje de IA Física.',
    thinking: 'Pensando...',
    online: 'En línea',
    clearChat: 'Limpiar conversación',
    send: 'Enviar',
    selectLanguage: 'Seleccionar Idioma',
    chapters: 'Capítulos',
    aiAssistant: 'Asistente IA',
  },
  ar: {
    dashboard: 'لوحة تحكم المتعلم',
    profile: 'ملفك الشخصي',
    difficulty: 'مستوى الصعوبة',
    progress: 'نظرة عامة على التقدم',
    assessments: 'نتائج التقييم الأخيرة',
    score: 'النتيجة',
    passed: 'ناجح',
    failed: 'راسب',
    feedback: 'ملاحظات',
    continue: 'متابعة التعلم',
    editProfile: 'تعديل الملف الشخصي',
    totalAssessments: 'إجمالي التقييمات',
    averageScore: 'متوسط النتيجة',
    experienceScores: 'درجات الخبرة',
    recommendedChapters: 'الفصول الموصى بها',
    python: 'بايثون',
    ml: 'التعلم الآلي',
    robotics: 'الروبوتات',
    ros: 'ROS 2',
    beginner: 'مبتدئ',
    intermediate: 'متوسط',
    advanced: 'متقدم',
    demoMode: 'وضع العرض - الخادم غير متصل. عرض بيانات نموذجية.',
    noResults: 'لا توجد نتائج تقييم بعد.',
    chatPlaceholder: 'اسأل عن ROS 2، الروبوتات، المحتوى...',
    chatWelcome: 'مرحبا! أنا مساعد تعلم الذكاء الاصطناعي الفيزيائي.',
    thinking: 'جاري التفكير...',
    online: 'متصل',
    clearChat: 'مسح المحادثة',
    send: 'إرسال',
    selectLanguage: 'اختر اللغة',
    chapters: 'الفصول',
    aiAssistant: 'مساعد AI',
  },
  hi: {
    dashboard: 'लर्नर डैशबोर्ड',
    profile: 'आपकी प्रोफाइल',
    difficulty: 'कठिनाई स्तर',
    progress: 'प्रगति अवलोकन',
    assessments: 'हाल के मूल्यांकन परिणाम',
    score: 'स्कोर',
    passed: 'उत्तीर्ण',
    failed: 'अनुत्तीर्ण',
    feedback: 'प्रतिक्रिया',
    continue: 'सीखना जारी रखें',
    editProfile: 'प्रोफाइल संपादित करें',
    totalAssessments: 'कुल मूल्यांकन',
    averageScore: 'औसत स्कोर',
    experienceScores: 'अनुभव स्कोर',
    recommendedChapters: 'अनुशंसित अध्याय',
    python: 'पायथन',
    ml: 'ML/AI',
    robotics: 'रोबोटिक्स',
    ros: 'ROS 2',
    beginner: 'शुरुआती',
    intermediate: 'मध्यवर्ती',
    advanced: 'उन्नत',
    demoMode: 'डेमो मोड - बैकएंड कनेक्ट नहीं। नमूना डेटा दिखा रहा है।',
    noResults: 'अभी तक कोई मूल्यांकन परिणाम नहीं।',
    chatPlaceholder: 'ROS 2, रोबोटिक्स, कोर्स के बारे में पूछें...',
    chatWelcome: 'नमस्ते! मैं आपका फिजिकल AI लर्निंग असिस्टेंट हूं।',
    thinking: 'सोच रहा हूं...',
    online: 'ऑनलाइन',
    clearChat: 'बातचीत साफ़ करें',
    send: 'भेजें',
    selectLanguage: 'भाषा चुनें',
    chapters: 'अध्याय',
    aiAssistant: 'AI सहायक',
  },
  fr: {
    dashboard: 'Tableau de Bord',
    profile: 'Votre Profil',
    difficulty: 'Niveau de Difficulté',
    progress: 'Aperçu des Progrès',
    assessments: 'Résultats Récents',
    score: 'Score',
    passed: 'RÉUSSI',
    failed: 'ÉCHOUÉ',
    feedback: 'Commentaires',
    continue: 'Continuer à Apprendre',
    editProfile: 'Modifier le Profil',
    totalAssessments: 'Total des Évaluations',
    averageScore: 'Score Moyen',
    experienceScores: "Scores d'Expérience",
    recommendedChapters: 'Chapitres Recommandés',
    python: 'Python',
    ml: 'ML/IA',
    robotics: 'Robotique',
    ros: 'ROS 2',
    beginner: 'Débutant',
    intermediate: 'Intermédiaire',
    advanced: 'Avancé',
    demoMode: 'Mode Démo - Backend non connecté. Affichage des données exemples.',
    noResults: "Pas encore de résultats d'évaluation.",
    chatPlaceholder: 'Posez des questions sur ROS 2, robotique, cours...',
    chatWelcome: 'Bonjour! Je suis votre assistant IA Physique.',
    thinking: 'Réflexion...',
    online: 'En ligne',
    clearChat: 'Effacer la conversation',
    send: 'Envoyer',
    selectLanguage: 'Sélectionner la Langue',
    chapters: 'Chapitres',
    aiAssistant: 'Assistant IA',
  },
};

// Language metadata
const LANGUAGES = [
  { code: 'en', label: 'English', flag: '🇺🇸', native: 'English', dir: 'ltr' },
  { code: 'ur', label: 'Urdu', flag: '🇵🇰', native: 'اردو', dir: 'rtl' },
  { code: 'zh', label: 'Chinese', flag: '🇨🇳', native: '中文', dir: 'ltr' },
  { code: 'es', label: 'Spanish', flag: '🇪🇸', native: 'Español', dir: 'ltr' },
  { code: 'ar', label: 'Arabic', flag: '🇸🇦', native: 'العربية', dir: 'rtl' },
  { code: 'hi', label: 'Hindi', flag: '🇮🇳', native: 'हिन्दी', dir: 'ltr' },
  { code: 'fr', label: 'French', flag: '🇫🇷', native: 'Français', dir: 'ltr' },
];

export const TranslationProvider = ({ children }) => {
  const [currentLang, setCurrentLang] = useState('en');
  const [isTranslating, setIsTranslating] = useState(false);

  // Get translation for a key
  const t = useCallback((key) => {
    const translations = UI_TRANSLATIONS[currentLang] || UI_TRANSLATIONS.en;
    return translations[key] || UI_TRANSLATIONS.en[key] || key;
  }, [currentLang]);

  // Change language
  const changeLanguage = useCallback((langCode) => {
    setCurrentLang(langCode);
    // Update document direction for RTL languages
    const lang = LANGUAGES.find(l => l.code === langCode);
    if (lang && typeof document !== 'undefined') {
      document.documentElement.dir = lang.dir;
      document.documentElement.lang = langCode;
    }
  }, []);

  // Get current language info
  const getCurrentLanguage = useCallback(() => {
    return LANGUAGES.find(l => l.code === currentLang) || LANGUAGES[0];
  }, [currentLang]);

  const value = {
    currentLang,
    changeLanguage,
    t,
    languages: LANGUAGES,
    getCurrentLanguage,
    isTranslating,
    isRTL: ['ar', 'ur'].includes(currentLang),
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

// Hook to use translation
export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    // Return default values if not in provider
    return {
      currentLang: 'en',
      changeLanguage: () => {},
      t: (key) => UI_TRANSLATIONS.en[key] || key,
      languages: LANGUAGES,
      getCurrentLanguage: () => LANGUAGES[0],
      isTranslating: false,
      isRTL: false,
    };
  }
  return context;
};

export default TranslationProvider;
