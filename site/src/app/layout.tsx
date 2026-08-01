import type { Metadata } from "next";
import { Provider } from "@/components/provider";
import { appDescription, appName, siteUrl } from "@/lib/shared";
import "katex/dist/katex.css";
// Inter Variable is loaded through `global.css` (@fontsource-variable/inter),
// not `next/font/google`, so the optical-size axis comes with it.
import "./global.css";

export const metadata: Metadata = {
  metadataBase: new URL(siteUrl),
  title: {
    default: appName,
    template: `%s | ${appName}`,
  },
  description: appDescription,
};

export default function Layout({ children }: LayoutProps<"/">) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className="flex flex-col min-h-screen">
        <Provider>{children}</Provider>
      </body>
    </html>
  );
}
