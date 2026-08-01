import { GithubInfo } from "fumadocs-ui/components/github-info";
import { DocsLayout } from "fumadocs-ui/layouts/docs";
import {
  AccordionFolder,
  SidebarAccordion,
} from "@/components/sidebar-accordion";
import { ThemeToggle } from "@/components/theme-toggle";
import { baseOptions } from "@/lib/layout.shared";
import { projectRepo } from "@/lib/shared";
import { source } from "@/lib/source";

/**
 * The search trigger is styled to read as another sidebar section rather than
 * an inset box: flush to both edges, square, with the same rule beneath it that
 * separates the sections.
 *
 * `FullSearchTrigger` merges this over its own classes with a tailwind-aware
 * `cn`, so the conflicting utilities (`rounded-lg`, `border`, `bg-…`) are
 * overridden rather than fought with. The negative inline margin cancels the
 * `p-4` on the sidebar header that would otherwise inset it.
 *
 * `ps-*` rather than `px-*`: the default `ps-2` survives the merge and would
 * win over any `px-*` override on the start edge. The value lines the magnifier
 * up with the section icons, so it tracks `FOLDER_CLASS`'s `px-2` plus the
 * trigger's own 0.5rem indent.
 */
const SEARCH_CLASS =
  // `border-y` on top of `border-0` keeps the sides off while ruling the row
  // above and below, so it reads as a band between the logo and the sections.
  "-mx-4 rounded-none border-0 border-y border-fd-border bg-transparent" +
  // `-mb-2` cancels the `pb-2` on the sidebar header, closing the gap to the
  // first section.
  " ps-4 pe-2 py-2 -mb-2 hover:bg-fd-accent/50" +
  // Match the section rows: same `text-sm` label, same 1rem icon.
  " text-sm [&_svg]:size-4" +
  // The shortcut keycaps inherit the label size, which is too heavy for them.
  " [&_kbd]:text-[0.625rem] [&_kbd]:px-1 [&_kbd]:py-px";

/**
 * Ruled off from the footer controls above it and scaled down to sit closer to
 * the sidebar rows. `-mx-4 px-4` cancels the footer's own `p-4` so the rule —
 * and the hover state — reach both edges, matching the sections.
 *
 * The component stacks the repo name over the counts (`flex-col`); `flex-row`
 * plus `justify-between` puts the stars and forks on the right instead. The
 * name keeps its own `truncate`, so `shrink-0` on the counts makes the name
 * give way first when the sidebar is narrow.
 */
const GITHUB_INFO_CLASS =
  // `-mb-4` cancels the footer container's own `pb-4` so the card sits flush
  // with the bottom of the sidebar.
  "-mx-4 px-4 py-3 mt-2 -mb-4 rounded-none border-t border-fd-border" +
  " flex-row items-center justify-between gap-2 [&>div]:shrink-0" +
  " text-[0.6875rem] [&_svg]:size-3 [&_div]:text-[0.625rem]";

export default function Layout({ children }: LayoutProps<"/docs">) {
  const base = baseOptions();

  return (
    <SidebarAccordion>
      <DocsLayout
        tree={source.getPageTree()}
        sidebar={{
          // The aside defaults to `bg-fd-card`, a step off the page. This
          // merges through a tailwind-aware `cn`, so it replaces that rather
          // than layering on it; the `border-e` still divides the two.
          className: "bg-fd-background",
          collapsible: false,
          components: {
            Folder: AccordionFolder,
          },
          // Async server component: with `output: 'export'` the GitHub API call
          // happens at build time, so the counts are as fresh as the last
          // deploy. `GITHUB_TOKEN` is optional locally but matters in CI, where
          // shared runners hit the unauthenticated rate limit.
          footer: (
            <GithubInfo
              className={GITHUB_INFO_CLASS}
              owner={projectRepo.owner}
              repo={projectRepo.repo}
              token={process.env.GITHUB_TOKEN}
            />
          ),
        }}
        {...base}
        // The sidebar header renders `nav.children` straight after the title,
        // and the title carries `me-auto`, so this lands hard right of the logo.
        nav={{
          ...base.nav,
          children: <ThemeToggle />,
        }}
        // Overrides after the spread, so this applies to the docs sidebar only
        // and the home navbar keeps both controls. The GitHub icon is redundant
        // with the repo card in the footer; `themeSwitch` stays off so the
        // switcher does not also reappear in the footer.
        githubUrl={undefined}
        themeSwitch={{ enabled: false }}
        searchToggle={{
          full: {
            className: SEARCH_CLASS,
          },
        }}
      >
        {children}
      </DocsLayout>
    </SidebarAccordion>
  );
}
