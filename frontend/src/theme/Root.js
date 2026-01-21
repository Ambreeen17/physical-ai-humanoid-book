import React from 'react';
import { AuthProvider } from '../components/AuthProvider';
import { TranslationProvider } from '../components/TranslationProvider';

export default function Root({children}) {
  return (
    <AuthProvider>
      <TranslationProvider>
        {children}
      </TranslationProvider>
    </AuthProvider>
  );
}
