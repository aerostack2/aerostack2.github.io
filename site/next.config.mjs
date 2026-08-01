import { createMDX } from "fumadocs-mdx/next";

const withMDX = createMDX();

/** @type {import('next').NextConfig} */
const config = {
  output: "export",
  reactStrictMode: true,
  // `next/image` optimisation needs a server; GitHub Pages serves plain files.
  images: {
    unoptimized: true,
  },
};

export default withMDX(config);
