import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import LanguageSwitcher from '../../components/LanguageSwitcher';
import FloatingChatbot from '../../components/FloatingChatbot';
import AuthButtons from '../../components/AuthButtons';
import './navbar-custom.css';

export default function NavbarWrapper(props) {
  return (
    <div className="navbar-wrapper">
      <OriginalNavbar {...props} />
      <div className="navbar-extras">
        <LanguageSwitcher />
        <AuthButtons />
      </div>
      <FloatingChatbot />
    </div>
  );
}
