import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';
import NavbarColorModeToggle from './ColorModeToggle';

export default function Navbar(): React.JSX.Element {
  return (
    <NavbarLayout>
      <NavbarContent>
        <NavbarColorModeToggle />
      </NavbarContent>
    </NavbarLayout>
  );
}