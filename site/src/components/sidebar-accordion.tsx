"use client";

import type * as PageTree from "fumadocs-core/page-tree";
import * as Base from "fumadocs-ui/components/sidebar/base";
import { useTreePath } from "fumadocs-ui/contexts/tree";
import {
  type ComponentProps,
  createContext,
  type ReactNode,
  useContext,
  useMemo,
  useState,
} from "react";

interface AccordionState {
  /** `undefined` means nothing has been clicked yet, so follow the active page. */
  openId: string | null | undefined;
  setOpenId: (id: string | null) => void;
}

const AccordionContext = createContext<AccordionState | null>(null);

/**
 * Makes the sidebar sections behave as an accordion: opening one closes
 * whichever was open before.
 */
export function SidebarAccordion({ children }: { children: ReactNode }) {
  const [openId, setOpenId] = useState<string | null | undefined>(undefined);
  const value = useMemo<AccordionState>(
    () => ({ openId, setOpenId }),
    [openId],
  );

  return <AccordionContext value={value}>{children}</AccordionContext>;
}

/**
 * `SidebarFolder` renders `<Collapsible open onOpenChange {...props}>`, spreading
 * its extra props *after* the open state — so passing `open`/`onOpenChange`
 * through takes control of the Collapsible. Its prop type only declares div
 * attributes, hence the cast.
 *
 * This leans on an implementation detail: if a future Fumadocs stops forwarding
 * props to the Collapsible, the sections revert to opening independently rather
 * than failing loudly. The build will not catch that — check the behaviour after
 * upgrading `fumadocs-ui`.
 */
const ControlledFolder = Base.SidebarFolder as (
  props: ComponentProps<typeof Base.SidebarFolder> & {
    open?: boolean;
    onOpenChange?: (open: boolean) => void;
  },
) => ReactNode;

/**
 * DocsLayout wraps the base folder primitives in styled versions of its own,
 * but exports neither them nor the `cva` behind them. Overriding `Folder` means
 * rendering the base primitives directly, so these classes are copied verbatim
 * from `layouts/docs/slots/sidebar` (`itemVariants` + the `button` variant) to
 * keep the trigger looking like every other sidebar row. Without them the icon,
 * label and chevron lose their flex row and stack vertically.
 *
 * Deviations from the original: `p-2` is tightened to `px-2 py-1`, the
 * `hover:bg-fd-accent/50` fill is dropped in favour of the text colour alone,
 * and `cursor-pointer` is added — the trigger is a `<button>`, which browsers
 * give a default arrow cursor.
 */
const TRIGGER_CLASS =
  "relative flex flex-row items-center gap-2 rounded-lg px-2 py-1 text-start" +
  " text-fd-muted-foreground wrap-anywhere [&_svg]:size-4 [&_svg]:shrink-0" +
  " cursor-pointer transition-colors hover:text-fd-accent-foreground/80" +
  " hover:transition-none w-full";

/** Top-level folders sit at depth 1, whose offset is `calc(2 * var(--spacing))`. */
const TRIGGER_PADDING_START = "calc(2 * var(--spacing))";

/**
 * Rule between sections. `-mx-4` cancels the `p-4` on the sidebar viewport so
 * the border reaches both edges; `px-2` then sets the inset for the contents.
 * Because the border sits outside the padding, this controls how far the
 * triggers and page rows sit from the sidebar edges without affecting the rule.
 * Keep the search trigger's `ps`/`pe` in step with it, or the magnifier stops
 * lining up with the section icons.
 *
 * `first:` clears the rule on the top section. That holds only while
 * `baseOptions()` defines no `links`, since those render as siblings above the
 * page tree in the same flex column.
 */
const FOLDER_CLASS =
  "-mx-4 px-2 border-t border-fd-border pt-1 mt-1" +
  // The first section also cancels the viewport's `pt-4` so it sits just under
  // the search box rather than a full row below it.
  " first:border-t-0 first:pt-1 first:-mt-4";

/** `[&>a]:py-1` tightens the page rows, which default to `p-2`. */
const CONTENT_CLASS =
  "relative flex flex-col pt-0.5 pb-1 [&>a]:py-1 before:content-[''] " +
  "before:absolute before:w-px before:inset-y-1 before:bg-fd-border" +
  " before:inset-s-2.5";

/**
 * Drop-in for the sidebar's default folder renderer. Mirrors the markup that
 * `createPageTreeRenderer` produces, but drives the open state from the
 * accordion context instead of each folder's own `useState`.
 */
export function AccordionFolder({
  item,
  children,
}: {
  item: PageTree.Folder;
  children: ReactNode;
}) {
  const accordion = useContext(AccordionContext);
  const path = useTreePath();
  const active = path.includes(item);
  const id = String(item.$id ?? item.name);

  const open =
    accordion?.openId === undefined ? active : accordion.openId === id;

  return (
    <ControlledFolder
      className={FOLDER_CLASS}
      collapsible={item.collapsible}
      active={active}
      open={open}
      onOpenChange={(next: boolean) => accordion?.setOpenId(next ? id : null)}
    >
      <Base.SidebarFolderTrigger
        className={TRIGGER_CLASS}
        style={{ paddingInlineStart: TRIGGER_PADDING_START }}
      >
        {item.icon}
        {item.name}
      </Base.SidebarFolderTrigger>
      <Base.SidebarFolderContent className={CONTENT_CLASS}>
        {children}
      </Base.SidebarFolderContent>
    </ControlledFolder>
  );
}
