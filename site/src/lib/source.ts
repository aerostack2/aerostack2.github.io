import { loader } from "fumadocs-core/source";
import { metaSchema, pageSchema } from "fumadocs-core/source/schema";
import { applyMdxPreset } from "fumadocs-mdx/config";
import { defineDocs } from "fumadocs-mdx/macro";
import * as lucide from "lucide-react";
import { createElement } from "react";
import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";
import { docsContentRoute, docsImageRoute, docsRoute } from "./shared";

const docs = defineDocs({
  dir: "content/docs",
  docs: {
    schema: pageSchema,
    // A collection-level `mdxOptions` object would drop Fumadocs' own plugins,
    // so extend the preset instead. rehype-katex must run before the syntax
    // highlighter.
    mdxOptions: applyMdxPreset({
      remarkPlugins: (v) => [...v, remarkMath],
      rehypePlugins: (v) => [rehypeKatex, ...v],
    }),
    postprocess: {
      includeProcessedMarkdown: true,
    },
  },
  meta: {
    schema: metaSchema,
  },
});

// See https://fumadocs.dev/docs/headless/source-api for more info
export const source = loader({
  baseUrl: docsRoute,
  source: docs.toFumadocsSource(),
  plugins: [],
  // Resolves the `icon:` name in page and meta frontmatter to a lucide icon.
  icon(name) {
    if (!name) return;
    // Icons are `forwardRef` objects, not functions, so check membership.
    if (!(name in lucide)) {
      throw new Error(`Unknown lucide icon in frontmatter: ${name}`);
    }
    const Icon = (lucide as unknown as Record<string, lucide.LucideIcon>)[name];
    return createElement(Icon);
  },
});

export function getPageImageUrl(page: (typeof source)["$inferPage"]) {
  const segments = [...page.slugs, "image.png"];

  return {
    segments,
    url:
      "/" +
      [page.locale, ...docsImageRoute.split("/"), ...segments]
        .filter(Boolean)
        .join("/"),
  };
}

export function getPageMarkdownUrl(page: (typeof source)["$inferPage"]) {
  const segments = [...page.slugs, "content.md"];

  return {
    segments,
    url:
      "/" +
      [page.locale, ...docsContentRoute.split("/"), ...segments]
        .filter(Boolean)
        .join("/"),
  };
}

export async function getLLMText(page: (typeof source)["$inferPage"]) {
  const processed = await page.data.getText("processed");

  return `# ${page.data.title} (${page.url})

${processed}`;
}
