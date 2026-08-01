import { cn } from "@/lib/cn";

export interface VideoProps {
  /** YouTube video id, e.g. `IvItNtSB-QE`. */
  youtube?: string;
  /** Vimeo video id, e.g. `879000655`. */
  vimeo?: string;
  /** Full embed URL, when neither shorthand fits. */
  src?: string;
  title?: string;
  className?: string;
}

/**
 * Replacement for the hand-written `.. raw:: html` iframe embeds: a responsive
 * 16:9 video player.
 */
export function Video({ youtube, vimeo, src, title, className }: VideoProps) {
  const url = youtube
    ? `https://www.youtube.com/embed/${youtube}`
    : vimeo
      ? `https://player.vimeo.com/video/${vimeo}`
      : src;

  if (!url)
    throw new Error("<Video /> needs one of `youtube`, `vimeo` or `src`.");

  return (
    <div
      className={cn(
        "my-6 overflow-hidden rounded-lg border bg-fd-card",
        className,
      )}
    >
      <iframe
        src={url}
        title={title ?? "Video"}
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
        allowFullScreen
        referrerPolicy="strict-origin-when-cross-origin"
        className="aspect-video w-full"
      />
    </div>
  );
}
