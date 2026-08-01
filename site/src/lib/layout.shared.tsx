import type { BaseLayoutProps } from "fumadocs-ui/layouts/shared";
import Image from "next/image";
import { appName, gitConfig } from "./shared";

export function baseOptions(): BaseLayoutProps {
  return {
    nav: {
      title: (
        <>
          <Image
            src="/logo.png"
            alt=""
            width={22}
            height={22}
            className="rounded-sm"
          />
          <span className="font-medium">{appName}</span>
        </>
      ),
    },
    // The provider already defaults to `system`; without this the switcher only
    // offers light/dark, so there is no way back to it once a user picks one.
    themeSwitch: {
      mode: "light-dark-system",
    },
    // No `links`: GlassLayout renders them into the sidebar, right under the
    // tab dropdown that already switches between the same root sections.
    githubUrl: `https://github.com/${gitConfig.user}/${gitConfig.repo}`,
  };
}
