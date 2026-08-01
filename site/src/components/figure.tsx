import type { ReactNode } from "react";
import { cn } from "@/lib/cn";

export interface FigureProps {
  src: string;
  alt?: string;
  caption?: ReactNode;
  /** Width as a percentage of the content column, mirroring reST `:scale:`. */
  scale?: number;
  className?: string;
}

/**
 * Replacement for the reST `.. figure::` directive: an image with an optional
 * caption underneath.
 */
export function Figure({ src, alt, caption, scale, className }: FigureProps) {
  return (
    <figure className={cn("my-6 flex flex-col items-center gap-3", className)}>
      {/* biome-ignore lint/performance/noImgElement: sources are static assets and remote URLs */}
      <img
        src={src}
        alt={alt ?? (typeof caption === "string" ? caption : "")}
        style={scale ? { width: `${scale}%` } : undefined}
        className="rounded-lg border bg-fd-card"
      />
      {caption ? (
        <figcaption className="text-center text-sm text-fd-muted-foreground">
          {caption}
        </figcaption>
      ) : null}
    </figure>
  );
}
