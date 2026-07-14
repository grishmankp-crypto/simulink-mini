import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "simulink-mini editor",
  description: "Block-diagram editor for the simulink-mini compiler/solver/tuner",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="h-full antialiased">
      <body className="min-h-full flex flex-col font-sans">{children}</body>
    </html>
  );
}
